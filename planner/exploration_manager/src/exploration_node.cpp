#include <ros/ros.h>
#include "exploration_manager/exploration_fsm.h"

using namespace exploration_manager;

int main(int argc, char** argv) {
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");
  
  ROS_INFO("========================================");
  ROS_INFO("       Exploration Node Started         ");
  ROS_INFO("========================================");
  
  ExplorationFSM fsm;
  fsm.init(nh);
  
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  
  return 0;
}
