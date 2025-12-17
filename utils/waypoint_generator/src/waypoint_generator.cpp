#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

using namespace std;

ros::Publisher pub_waypoints_;
ros::Publisher pub_waypoints_vis_;
std::string waypoint_type_ = "manual";
bool is_odom_ready_ = false;
nav_msgs::Odometry odom_;
nav_msgs::Path waypoints_;

void publish_waypoints() {
  waypoints_.header.frame_id = std::string("world");
  waypoints_.header.stamp = ros::Time::now();
  pub_waypoints_.publish(waypoints_);
  
  geometry_msgs::PoseStamped init_pose;
  init_pose.header = odom_.header;
  init_pose.pose = odom_.pose.pose;
  waypoints_.poses.insert(waypoints_.poses.begin(), init_pose);
  waypoints_.poses.clear();
}

void publish_waypoints_vis() {
  geometry_msgs::PoseArray poseArray;
  poseArray.header.frame_id = std::string("world");
  poseArray.header.stamp = ros::Time::now();

  {
    geometry_msgs::Pose init_pose;
    init_pose = odom_.pose.pose;
    poseArray.poses.push_back(init_pose);
  }

  for (auto it = waypoints_.poses.begin(); it != waypoints_.poses.end(); ++it) {
    geometry_msgs::Pose p;
    p = it->pose;
    poseArray.poses.push_back(p);
  }
  pub_waypoints_vis_.publish(poseArray);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  odom_ = *msg;
  is_odom_ready_ = true;
}

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (!is_odom_ready_) {
    ROS_WARN("[waypoint_generator] No odom yet, waiting...");
    return;
  }

  ros::NodeHandle nh("~");
  nh.param("waypoint_type", waypoint_type_, std::string("manual"));

  if (waypoint_type_ == std::string("point")) {
    // Single point mode - publish immediately
    if (msg->pose.position.z > -0.1) {
      geometry_msgs::PoseStamped pt = *msg;
      waypoints_.poses.clear();
      waypoints_.poses.push_back(pt);
      publish_waypoints_vis();
      publish_waypoints();
      ROS_INFO("[waypoint_generator] Goal published at (%.2f, %.2f, %.2f)", 
               pt.pose.position.x, pt.pose.position.y, pt.pose.position.z);
    }
  } else if (waypoint_type_ == std::string("manual")) {
    // Manual waypoint mode - collect multiple points
    if (msg->pose.position.z > 0) {
      // Height > 0: add waypoint
      geometry_msgs::PoseStamped pt = *msg;
      double yaw = tf::getYaw(odom_.pose.pose.orientation);
      pt.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      waypoints_.poses.push_back(pt);
      publish_waypoints_vis();
      ROS_INFO("[waypoint_generator] Waypoint added: (%.2f, %.2f, %.2f), total: %lu", 
               pt.pose.position.x, pt.pose.position.y, pt.pose.position.z, 
               waypoints_.poses.size());
    } else if (msg->pose.position.z > -1.0) {
      // 0 > height > -1.0: remove last waypoint
      if (waypoints_.poses.size() >= 1) {
        waypoints_.poses.erase(std::prev(waypoints_.poses.end()));
        ROS_INFO("[waypoint_generator] Last waypoint removed, remaining: %lu", 
                 waypoints_.poses.size());
      }
      publish_waypoints_vis();
    } else {
      // Height < -1.0: publish all waypoints
      if (waypoints_.poses.size() >= 1) {
        publish_waypoints_vis();
        publish_waypoints();
        ROS_INFO("[waypoint_generator] Published %lu waypoints!", waypoints_.poses.size());
      }
    }
  }
}

void traj_start_trigger_callback(const geometry_msgs::PoseStamped& msg) {
  if (!is_odom_ready_) {
    ROS_ERROR("[waypoint_generator] No odom!");
    return;
  }

  ROS_WARN("[waypoint_generator] Trigger received!");
  
  // Publish a single waypoint for exploration trigger
  if (waypoints_.poses.size() >= 1) {
    publish_waypoints_vis();
    publish_waypoints();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_generator");
  ros::NodeHandle nh("~");
  
  nh.param("waypoint_type", waypoint_type_, std::string("point"));
  
  ros::Subscriber sub_odom = nh.subscribe("odom", 10, odom_callback);
  ros::Subscriber sub_goal = nh.subscribe("goal", 10, goal_callback);
  ros::Subscriber sub_trigger = nh.subscribe("traj_start_trigger", 10, traj_start_trigger_callback);
  
  pub_waypoints_ = nh.advertise<nav_msgs::Path>("waypoints", 50);
  pub_waypoints_vis_ = nh.advertise<geometry_msgs::PoseArray>("waypoints_vis", 10);

  ROS_INFO("[waypoint_generator] Node started in '%s' mode", waypoint_type_.c_str());
  ROS_INFO("[waypoint_generator] Use '3D Nav Goal' tool in RViz to set goals");
  ROS_INFO("[waypoint_generator]   - Height > 0: Add waypoint (or trigger in 'point' mode)");
  ROS_INFO("[waypoint_generator]   - Height 0 to -1: Remove last waypoint");
  ROS_INFO("[waypoint_generator]   - Height < -1: Publish waypoints");

  ros::spin();
  return 0;
}
