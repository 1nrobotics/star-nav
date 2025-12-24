#include <ros/ros.h>
#include <memory>
#include "exploration_manager/waypoint_exploration.h"

using namespace exploration_manager;

int main(int argc, char** argv) {
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_global;

  ROS_INFO("========================================");
  ROS_INFO("       Waypoint Exploration Node        ");
  ROS_INFO("========================================");

  // Create waypoint exploration
  auto waypoint_exploration = std::make_shared<WaypointExploration>();
  waypoint_exploration->init(nh);

  ROS_INFO("Waypoint exploration initialized");

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
