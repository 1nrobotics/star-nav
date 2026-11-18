#include <ros/ros.h>
#include <memory>
#include <string>
#include "exploration_manager/frontier_exploration.h"
#include "exploration_manager/waypoint_exploration.h"

using namespace exploration_manager;

int main(int argc, char** argv) {
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");

  // frontier: explore the area's unknown space; waypoint: fly a lawnmower.
  std::string mode;
  nh.param("exploration/mode", mode, std::string("waypoint"));
  ROS_INFO("Exploration node, mode: %s", mode.c_str());

  if (mode == "frontier") {
    FrontierExploration exploration;
    exploration.init(nh);
    ros::spin();  // one thread: the map and the FSM share data
    return 0;
  }
  if (mode != "waypoint") {
    ROS_ERROR("Unknown exploration/mode '%s' (frontier or waypoint)", mode.c_str());
    return 1;
  }

  auto waypoint_exploration = std::make_shared<WaypointExploration>();
  waypoint_exploration->init(nh);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
