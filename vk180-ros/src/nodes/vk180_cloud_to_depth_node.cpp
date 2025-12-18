#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include <mutex>

namespace {

inline bool isFinite(double v) {
  return std::isfinite(v);
}

class CloudToDepth {
public:
  CloudToDepth(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    pnh_.param("width", width_, 640);
    pnh_.param("height", height_, 480);

    pnh_.param("fx", fx_, 320.0);
    pnh_.param("fy", fy_, 320.0);
    pnh_.param("cx", cx_, 320.0);
    pnh_.param("cy", cy_, 240.0);

    pnh_.param("min_depth", min_depth_, 0.2);
    pnh_.param("max_depth", max_depth_, 5.0);

    // If pose is older than this (seconds), warn (still publishes).
    pnh_.param("max_pose_age", max_pose_age_, 0.5);

    // If set > 1, skips points for speed (simple decimation).
    pnh_.param("point_stride", point_stride_, 1);

    // Output encoding is fixed to RealSense-like 16UC1 (millimeters).
    depth_pub_ = nh_.advertise<sensor_msgs::Image>("depth", 10);

    pose_sub_ = nh_.subscribe("pose", 50, &CloudToDepth::poseCallback, this);
    cloud_sub_ = nh_.subscribe("cloud", 10, &CloudToDepth::cloudCallback, this);

    if (width_ <= 0 || height_ <= 0) {
      ROS_FATAL("Invalid image size width=%d height=%d", width_, height_);
      throw std::runtime_error("invalid image size");
    }

    if (fx_ <= 0.0 || fy_ <= 0.0) {
      ROS_FATAL("Invalid intrinsics fx=%f fy=%f", fx_, fy_);
      throw std::runtime_error("invalid intrinsics");
    }

    if (point_stride_ < 1) point_stride_ = 1;

    ROS_INFO("vk180_cloud_to_depth: subscribing cloud=%s pose=%s; publishing depth=%s",
             cloud_sub_.getTopic().c_str(), pose_sub_.getTopic().c_str(), depth_pub_.getTopic().c_str());
    ROS_INFO("vk180_cloud_to_depth: intrinsics fx=%.3f fy=%.3f cx=%.3f cy=%.3f size=%dx%d",
             fx_, fy_, cx_, cy_, width_, height_);
    ROS_INFO("vk180_cloud_to_depth: depth range [%.2f, %.2f] m, point_stride=%d",
             min_depth_, max_depth_, point_stride_);
  }

private:
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    last_pose_ = *pose_msg;
    has_pose_ = true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    geometry_msgs::PoseStamped pose_msg;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      if (!has_pose_) {
        ROS_WARN_THROTTLE(2.0, "vk180_cloud_to_depth: no pose received yet; skipping cloud");
        return;
      }
      pose_msg = last_pose_;
    }

    const ros::Time cloud_stamp = (cloud_msg->header.stamp.isZero() ? ros::Time::now() : cloud_msg->header.stamp);
    const double pose_age = (cloud_stamp - pose_msg.header.stamp).toSec();
    if (max_pose_age_ > 0.0 && std::fabs(pose_age) > max_pose_age_) {
      ROS_WARN_THROTTLE(2.0, "vk180_cloud_to_depth: pose age %.3fs exceeds max_pose_age=%.3fs", pose_age,
                        max_pose_age_);
    }

    // Build world->camera transform from pose (camera in world).
    const auto& p = pose_msg.pose.position;
    const auto& q = pose_msg.pose.orientation;

    Eigen::Quaterniond q_w_c(q.w, q.x, q.y, q.z);
    if (q_w_c.norm() == 0.0) {
      ROS_WARN_THROTTLE(2.0, "Pose quaternion has zero norm; skipping frame");
      return;
    }
    q_w_c.normalize();

    const Eigen::Matrix3d R_w_c = q_w_c.toRotationMatrix();
    const Eigen::Vector3d t_w_c(p.x, p.y, p.z);

    // Prepare output depth image (16UC1, mm).
    sensor_msgs::Image depth;
    depth.header.stamp = pose_msg.header.stamp;
    depth.header.frame_id = "camera_depth_optical_frame";
    depth.height = static_cast<uint32_t>(height_);
    depth.width = static_cast<uint32_t>(width_);
    depth.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    depth.is_bigendian = false;
    depth.step = static_cast<uint32_t>(width_ * sizeof(uint16_t));
    depth.data.assign(static_cast<size_t>(height_ * depth.step), 0);

    uint16_t* depth_ptr = reinterpret_cast<uint16_t*>(depth.data.data());

    // Iterate points
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

    const size_t n = cloud_msg->width * cloud_msg->height;
    size_t kept = 0;
    size_t projected = 0;

    for (size_t i = 0; i < n; ++i, ++iter_x, ++iter_y, ++iter_z) {
      if (point_stride_ > 1 && (i % static_cast<size_t>(point_stride_) != 0)) {
        continue;
      }

      const double xw = static_cast<double>(*iter_x);
      const double yw = static_cast<double>(*iter_y);
      const double zw = static_cast<double>(*iter_z);

      if (!isFinite(xw) || !isFinite(yw) || !isFinite(zw)) {
        continue;
      }

      kept++;

      const Eigen::Vector3d p_w(xw, yw, zw);
      const Eigen::Vector3d p_c = R_w_c.transpose() * (p_w - t_w_c);

      const double xc = p_c.x();
      const double yc = p_c.y();
      const double zc = p_c.z();

      if (!isFinite(zc) || zc <= 0.0) {
        continue;
      }

      if (zc < min_depth_ || zc > max_depth_) {
        continue;
      }

      const int u = static_cast<int>(std::lround(fx_ * (xc / zc) + cx_));
      const int v = static_cast<int>(std::lround(fy_ * (yc / zc) + cy_));

      if (u < 0 || u >= width_ || v < 0 || v >= height_) {
        continue;
      }

      const uint16_t depth_mm = static_cast<uint16_t>(std::min(65535.0, std::max(0.0, zc * 1000.0)));
      const size_t idx = static_cast<size_t>(v) * static_cast<size_t>(width_) + static_cast<size_t>(u);
      uint16_t& prev = depth_ptr[idx];

      if (prev == 0 || depth_mm < prev) {
        prev = depth_mm;
      }

      projected++;
    }

    ROS_INFO_THROTTLE(2.0, "vk180_cloud_to_depth: points=%zu kept=%zu projected=%zu", n, kept, projected);

    depth_pub_.publish(depth);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber pose_sub_;

  ros::Publisher depth_pub_;

  std::mutex pose_mutex_;
  geometry_msgs::PoseStamped last_pose_;
  bool has_pose_ = false;

  int width_ = 640;
  int height_ = 480;
  double fx_ = 320.0;
  double fy_ = 320.0;
  double cx_ = 320.0;
  double cy_ = 240.0;
  double min_depth_ = 0.2;
  double max_depth_ = 5.0;
  double max_pose_age_ = 0.5;
  int point_stride_ = 1;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "vk180_cloud_to_depth");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    CloudToDepth node(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("vk180_cloud_to_depth: fatal error: %s", e.what());
    return 1;
  }

  return 0;
}
