// Thin ROS bridge for the standalone MuJoCo simulator.
//
// Receives state, depth and world messages from the simulator over ZeroMQ and
// republishes them with the topics and conventions of the original kinematic
// simulator, and forwards the planner's position command back.
//
//   simulator --state--> nav_msgs/Odometry          odom
//             --depth--> sensor_msgs/Image (16UC1)  depth
//                        geometry_msgs/PoseStamped  camera_pose
//             --lidar--> sensor_msgs/PointCloud2    lidar_cloud (world frame)
//                        geometry_msgs/PoseStamped  lidar_pose
//             --world--> sensor_msgs/PointCloud2    global_cloud (latched)
//                        visualization_msgs/MarkerArray  obstacles (latched,
//                        exact geometry for RViz and accuracy tests)
//                        std_msgs/UInt32            contacts
//   simulator <--cmd---- quadrotor_msgs/PositionCommand  pos_cmd
//
// With /use_sim_time the bridge also publishes /clock from the simulator, so
// the planning stack runs on simulated time and stays consistent with the
// physics even when the simulator runs slower than real time.

#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <zmq.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/UInt32.h>
#include <visualization_msgs/MarkerArray.h>

#include "protocol.h"

namespace mujoco_bridge {

class Bridge {
 public:
  explicit Bridge(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("state_address", state_address_, "tcp://127.0.0.1:5555");
    pnh.param<std::string>("cmd_address", cmd_address_, "tcp://127.0.0.1:5556");
    pnh.param<std::string>("world_frame", world_frame_, "world");
    pnh.param<std::string>("camera_frame", camera_frame_, "camera");
    pnh.param("cloud_resolution", cloud_resolution_, 0.1);
    // Ring geometry in the simulator (scene.RING_SEGMENTS, scene.RING_TUBE).
    pnh.param("ring_segments", ring_segments_, 24);
    pnh.param("ring_tube", ring_tube_, 0.04);

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);
    depth_pub_ = nh.advertise<sensor_msgs::Image>("depth", 5);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("camera_pose", 5);
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1, true);
    contacts_pub_ = nh.advertise<std_msgs::UInt32>("contacts", 1, true);
    obstacles_pub_ = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 1, true);
    lidar_pub_ = nh.advertise<sensor_msgs::PointCloud2>("lidar_cloud", 5);
    lidar_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("lidar_pose", 5);
    sim_time_ = ros::Time::isSimTime();
    if (sim_time_) clock_pub_ = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);
    cmd_sub_ = nh.subscribe("pos_cmd", 10, &Bridge::onCommand, this,
                            ros::TransportHints().tcpNoDelay());

    ctx_ = zmq_ctx_new();
    sub_ = zmq_socket(ctx_, ZMQ_SUB);
    for (const char* topic : {kTopicState, kTopicDepth, kTopicLidar, kTopicWorld}) {
      zmq_setsockopt(sub_, ZMQ_SUBSCRIBE, topic, std::strlen(topic));
    }
    int hwm = 20;
    zmq_setsockopt(sub_, ZMQ_RCVHWM, &hwm, sizeof(hwm));
    pub_ = zmq_socket(ctx_, ZMQ_PUB);
    if (zmq_connect(sub_, state_address_.c_str()) != 0 ||
        zmq_connect(pub_, cmd_address_.c_str()) != 0) {
      ROS_FATAL("[mujoco_bridge] cannot connect: %s", zmq_strerror(zmq_errno()));
      ros::shutdown();
      return;
    }
    ROS_INFO("[mujoco_bridge] simulator at %s, commands to %s",
             state_address_.c_str(), cmd_address_.c_str());
    running_ = true;
    thread_ = std::thread(&Bridge::receiveLoop, this);
  }

  ~Bridge() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    {
      std::lock_guard<std::mutex> lock(pub_mutex_);
      zmq_close(pub_);
    }
    zmq_close(sub_);
    zmq_ctx_term(ctx_);
  }

 private:
  // -- simulator -> ROS ------------------------------------------------------

  void receiveLoop() {
    zmq_pollitem_t item{sub_, 0, ZMQ_POLLIN, 0};
    std::vector<char> topic(16), payload;
    ros::WallTime last_rx = ros::WallTime::now();
    bool warned = false;
    while (running_ && ros::ok()) {
      if (zmq_poll(&item, 1, 100) <= 0) {
        if (!warned && (ros::WallTime::now() - last_rx).toSec() > 3.0) {
          ROS_WARN("[mujoco_bridge] no data from the simulator at %s yet",
                   state_address_.c_str());
          warned = true;
        }
        continue;
      }
      if (!receive(topic, payload)) continue;
      last_rx = ros::WallTime::now();
      warned = false;
      const std::string name(topic.data());
      if (name == kTopicState) {
        onState(payload);
      } else if (name == kTopicDepth) {
        onDepth(payload);
      } else if (name == kTopicLidar) {
        onLidar(payload);
      } else if (name == kTopicWorld) {
        onWorld(payload);
      }
    }
  }

  // Reads one two-frame message. Returns false on malformed input.
  bool receive(std::vector<char>& topic, std::vector<char>& payload) {
    zmq_msg_t frame;
    zmq_msg_init(&frame);
    if (zmq_msg_recv(&frame, sub_, 0) < 0) return false;
    const size_t n = std::min<size_t>(zmq_msg_size(&frame), topic.size() - 1);
    std::memcpy(topic.data(), zmq_msg_data(&frame), n);
    topic[n] = '\0';
    bool more = zmq_msg_more(&frame);
    zmq_msg_close(&frame);
    if (!more) return false;
    zmq_msg_init(&frame);
    zmq_msg_recv(&frame, sub_, 0);
    payload.assign(static_cast<char*>(zmq_msg_data(&frame)),
                   static_cast<char*>(zmq_msg_data(&frame)) + zmq_msg_size(&frame));
    more = zmq_msg_more(&frame);
    zmq_msg_close(&frame);
    while (more) {  // drain unexpected extra frames
      zmq_msg_init(&frame);
      zmq_msg_recv(&frame, sub_, 0);
      more = zmq_msg_more(&frame);
      zmq_msg_close(&frame);
    }
    return true;
  }

  template <typename T>
  bool checkHeader(const std::vector<char>& payload, const char* what, T* out) {
    if (payload.size() < sizeof(T)) {
      ROS_WARN_THROTTLE(5.0, "[mujoco_bridge] short %s message", what);
      return false;
    }
    std::memcpy(out, payload.data(), sizeof(T));
    if (out->version != kProtocolVersion) {
      ROS_ERROR_THROTTLE(5.0, "[mujoco_bridge] %s protocol version %u, expected %u",
                         what, out->version, kProtocolVersion);
      return false;
    }
    return true;
  }

  // Maps simulator time onto ROS time. With /use_sim_time the simulator is the
  // clock; otherwise it is assumed to run in real time and is offset to now.
  ros::Time stamp(double sim_t) {
    if (sim_time_) {
      const ros::Time t(sim_t);
      const bool jumped_back = t + ros::Duration(1.0) < last_clock_;
      if (jumped_back) ROS_WARN("[mujoco_bridge] simulator time jumped back (restarted?)");
      if (t > last_clock_ || jumped_back) {
        rosgraph_msgs::Clock clock;
        clock.clock = t;
        clock_pub_.publish(clock);
        last_clock_ = t;
      }
      return t;
    }
    const double now = ros::Time::now().toSec();
    if (!have_offset_ || std::fabs(now - (sim_t + offset_)) > 0.2) {
      if (have_offset_) ROS_WARN("[mujoco_bridge] resynchronising simulator clock");
      offset_ = now - sim_t;
      have_offset_ = true;
    }
    return ros::Time(sim_t + offset_);
  }

  void onState(const std::vector<char>& payload) {
    StateMsg s;
    if (!checkHeader(payload, "state", &s)) return;
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp(s.t);
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = "base";
    odom.pose.pose.position.x = s.pos[0];
    odom.pose.pose.position.y = s.pos[1];
    odom.pose.pose.position.z = s.pos[2];
    odom.pose.pose.orientation.w = s.quat[0];
    odom.pose.pose.orientation.x = s.quat[1];
    odom.pose.pose.orientation.y = s.quat[2];
    odom.pose.pose.orientation.z = s.quat[3];
    // Linear velocity in the world frame, like the kinematic simulator.
    odom.twist.twist.linear.x = s.vel[0];
    odom.twist.twist.linear.y = s.vel[1];
    odom.twist.twist.linear.z = s.vel[2];
    odom.twist.twist.angular.x = s.omega[0];
    odom.twist.twist.angular.y = s.omega[1];
    odom.twist.twist.angular.z = s.omega[2];
    odom_pub_.publish(odom);

    if (s.contacts != last_contacts_) {
      if (s.contacts > last_contacts_) {
        ROS_WARN_THROTTLE(1.0, "[mujoco_bridge] quadrotor collision (%u contact steps)",
                          s.contacts);
      }
      last_contacts_ = s.contacts;
      std_msgs::UInt32 c;
      c.data = s.contacts;
      contacts_pub_.publish(c);
    }
  }

  void onDepth(const std::vector<char>& payload) {
    DepthHeader h;
    if (!checkHeader(payload, "depth", &h)) return;
    const size_t bytes = size_t(h.width) * h.height * sizeof(uint16_t);
    if (payload.size() != sizeof(h) + bytes) {
      ROS_WARN_THROTTLE(5.0, "[mujoco_bridge] depth size mismatch");
      return;
    }
    const ros::Time t = stamp(h.t);
    sensor_msgs::ImagePtr img(new sensor_msgs::Image);
    img->header.stamp = t;
    img->header.frame_id = camera_frame_;
    img->width = h.width;
    img->height = h.height;
    img->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    img->is_bigendian = 0;
    img->step = h.width * sizeof(uint16_t);
    img->data.assign(payload.begin() + sizeof(h), payload.end());

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = t;
    pose.header.frame_id = world_frame_;
    pose.pose.position.x = h.pos[0];
    pose.pose.position.y = h.pos[1];
    pose.pose.position.z = h.pos[2];
    pose.pose.orientation.w = h.quat[0];
    pose.pose.orientation.x = h.quat[1];
    pose.pose.orientation.y = h.quat[2];
    pose.pose.orientation.z = h.quat[3];

    pose_pub_.publish(pose);
    depth_pub_.publish(img);
  }

  // Publishes the scan in the world frame, transformed with the sensor pose at
  // the time of the scan (what a lidar-inertial odometry front end provides).
  void onLidar(const std::vector<char>& payload) {
    LidarHeader h;
    if (!checkHeader(payload, "lidar", &h)) return;
    if (payload.size() != sizeof(h) + size_t(h.count) * 3 * sizeof(float)) {
      ROS_WARN_THROTTLE(5.0, "[mujoco_bridge] lidar size mismatch");
      return;
    }
    const ros::Time t = stamp(h.t);
    const double w = h.quat[0], x = h.quat[1], y = h.quat[2], z = h.quat[3];
    const double r[3][3] = {
        {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
        {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
        {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}};

    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = t;
    cloud.header.frame_id = world_frame_;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(h.count);
    sensor_msgs::PointCloud2Iterator<float> it(cloud, "x");
    const char* data = payload.data() + sizeof(h);
    for (uint32_t i = 0; i < h.count; ++i, ++it) {
      float p[3];
      std::memcpy(p, data + i * sizeof(p), sizeof(p));
      for (int k = 0; k < 3; ++k) {
        it[k] = float(h.pos[k] + r[k][0] * p[0] + r[k][1] * p[1] + r[k][2] * p[2]);
      }
    }

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = t;
    pose.header.frame_id = world_frame_;
    pose.pose.position.x = h.pos[0];
    pose.pose.position.y = h.pos[1];
    pose.pose.position.z = h.pos[2];
    pose.pose.orientation.w = w;
    pose.pose.orientation.x = x;
    pose.pose.orientation.y = y;
    pose.pose.orientation.z = z;

    lidar_pose_pub_.publish(pose);
    lidar_pub_.publish(cloud);
  }

  void onWorld(const std::vector<char>& payload) {
    WorldHeader h;
    if (!checkHeader(payload, "world", &h)) return;
    if (payload.size() != sizeof(h) + h.count * sizeof(ObstacleMsg)) {
      ROS_WARN_THROTTLE(5.0, "[mujoco_bridge] world size mismatch");
      return;
    }
    if (payload == last_world_) return;  // unchanged; cloud is latched
    last_world_ = payload;

    std::vector<float> pts;  // x, y, z triples
    const double res = cloud_resolution_;
    for (uint32_t i = 0; i < h.count; ++i) {
      ObstacleMsg o;
      std::memcpy(&o, payload.data() + sizeof(h) + i * sizeof(o), sizeof(o));
      if (o.kind == kPillar) {
        const int n = int(std::ceil(o.a / res));
        for (int ix = -n; ix <= n; ++ix) {
          for (int iy = -n; iy <= n; ++iy) {
            const double dx = ix * res, dy = iy * res;
            if (dx * dx + dy * dy > o.a * o.a) continue;
            for (double z = 0.5 * res; z < o.b; z += res) {
              pts.insert(pts.end(), {float(o.x + dx), float(o.y + dy), float(z)});
            }
          }
        }
      } else if (o.kind == kRing) {
        const double c = std::cos(o.yaw), s = std::sin(o.yaw);
        for (double ang = 0.0; ang < 2.0 * M_PI; ang += 0.5 * res) {
          const double ly = o.a * std::cos(ang), lz = o.b * std::sin(ang);
          pts.insert(pts.end(), {float(o.x - s * ly), float(o.y + c * ly),
                                 float(o.z + lz)});
        }
      }
    }

    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = world_frame_;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size() / 3);
    sensor_msgs::PointCloud2Iterator<float> it(cloud, "x");
    for (size_t i = 0; i < pts.size(); i += 3, ++it) {
      it[0] = pts[i];
      it[1] = pts[i + 1];
      it[2] = pts[i + 2];
    }
    cloud_pub_.publish(cloud);
    publishObstacles(payload, h.count);
    ROS_INFO("[mujoco_bridge] world: %u obstacles, %zu cloud points", h.count,
             pts.size() / 3);
  }

  // Pillars as cylinders; rings as the same closed polyline of capsules the
  // simulator builds (marker width = tube diameter).
  void publishObstacles(const std::vector<char>& payload, uint32_t count) {
    visualization_msgs::MarkerArray arr;
    for (uint32_t i = 0; i < count; ++i) {
      ObstacleMsg o;
      std::memcpy(&o, payload.data() + sizeof(WorldHeader) + i * sizeof(o), sizeof(o));
      visualization_msgs::Marker m;
      m.header.frame_id = world_frame_;
      m.header.stamp = ros::Time::now();
      m.id = int(i);
      m.action = visualization_msgs::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.color.a = 1.0;
      if (o.kind == kPillar) {
        m.ns = "pillar";
        m.type = visualization_msgs::Marker::CYLINDER;
        m.pose.position.x = o.x;
        m.pose.position.y = o.y;
        m.pose.position.z = o.b / 2.0;
        m.scale.x = m.scale.y = 2.0 * o.a;
        m.scale.z = o.b;
        m.color.r = 0.35, m.color.g = 0.55, m.color.b = 0.3;
      } else {
        m.ns = "ring";
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.scale.x = 2.0 * ring_tube_;
        m.color.r = 0.85, m.color.g = 0.55, m.color.b = 0.2;
        const double c = std::cos(o.yaw), s = std::sin(o.yaw);
        for (int k = 0; k <= ring_segments_; ++k) {
          const double ang = 2.0 * M_PI * k / ring_segments_;
          const double ly = o.a * std::cos(ang), lz = o.b * std::sin(ang);
          geometry_msgs::Point p;
          p.x = o.x - s * ly;
          p.y = o.y + c * ly;
          p.z = o.z + lz;
          m.points.push_back(p);
        }
      }
      arr.markers.push_back(m);
    }
    obstacles_pub_.publish(arr);
  }

  // -- ROS -> simulator ------------------------------------------------------

  void onCommand(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    CmdMsg c{};
    c.version = kProtocolVersion;
    c.t = msg->header.stamp.toSec();
    c.pos[0] = msg->position.x;
    c.pos[1] = msg->position.y;
    c.pos[2] = msg->position.z;
    c.vel[0] = msg->velocity.x;
    c.vel[1] = msg->velocity.y;
    c.vel[2] = msg->velocity.z;
    c.acc[0] = msg->acceleration.x;
    c.acc[1] = msg->acceleration.y;
    c.acc[2] = msg->acceleration.z;
    c.yaw = msg->yaw;
    c.yaw_rate = msg->yaw_dot;
    std::lock_guard<std::mutex> lock(pub_mutex_);
    zmq_send(pub_, kTopicCmd, std::strlen(kTopicCmd), ZMQ_SNDMORE);
    zmq_send(pub_, &c, sizeof(c), 0);
  }

  std::string state_address_, cmd_address_, world_frame_, camera_frame_;
  double cloud_resolution_ = 0.1;

  ros::Publisher odom_pub_, depth_pub_, pose_pub_, cloud_pub_, contacts_pub_, clock_pub_;
  ros::Publisher lidar_pub_, lidar_pose_pub_, obstacles_pub_;
  int ring_segments_ = 24;
  double ring_tube_ = 0.04;
  ros::Subscriber cmd_sub_;

  void* ctx_ = nullptr;
  void* sub_ = nullptr;
  void* pub_ = nullptr;
  std::mutex pub_mutex_;
  std::atomic<bool> running_{false};
  std::thread thread_;

  bool sim_time_ = false;
  ros::Time last_clock_;
  bool have_offset_ = false;
  double offset_ = 0.0;
  uint32_t last_contacts_ = UINT32_MAX;  // publish the first value
  std::vector<char> last_world_;
};

}  // namespace mujoco_bridge

int main(int argc, char** argv) {
  ros::init(argc, argv, "mujoco_bridge");
  ros::NodeHandle nh, pnh("~");
  mujoco_bridge::Bridge bridge(nh, pnh);
  ros::spin();
  return 0;
}
