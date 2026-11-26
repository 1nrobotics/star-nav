#ifndef _WAYPOINT_EXPLORATION_H_
#define _WAYPOINT_EXPLORATION_H_

#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

using std::shared_ptr;
using std::vector;
using std::string;

namespace exploration_manager {

class WaypointExploration {
public:
  WaypointExploration() {}
  ~WaypointExploration() {}

  void init(ros::NodeHandle& nh);
  void startExploration();
  void stopExploration();

  // ROS callbacks
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void finishCallback(const std_msgs::EmptyConstPtr& msg);

  // Main FSM loop
  void fsmCallback(const ros::TimerEvent& e);

private:
  enum EXPL_STATE {
    INIT,
    WAIT_TRIGGER,
    GENERATE_WAYPOINTS,
    PLAN_TO_WAYPOINT,
    EXEC_WAYPOINT,
    FINISH
  };

  // State handlers
  void initState();
  void waitTriggerState();
  void generateWaypointsState();
  void planToWaypointState();
  void execWaypointState();
  void finishState();

  // Utility functions
  void transitState(EXPL_STATE new_state, std::string pos_call);
  void visualizeState();
  void publishWaypoints();

  // Waypoint generation
  void generateLawnmowerWaypoints();
  bool isValidWaypoint(const Eigen::Vector3d& pos);

  // ROS
  ros::NodeHandle nh_;
  ros::Timer fsm_timer_;
  ros::Publisher goal_pub_;
  ros::Publisher state_pub_;
  ros::Publisher params_pub_;
  ros::Publisher waypoints_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber trigger_sub_;
  ros::Subscriber finish_sub_;

  // State
  EXPL_STATE state_;
  bool exploration_triggered_;
  ros::Time exploration_start_time_;

  // Odometry
  bool have_odom_;
  Eigen::Vector3d odom_pos_;
  Eigen::Vector3d odom_vel_;
  Eigen::Quaterniond odom_orient_;

  // Waypoints (position + yaw angle)
  vector<std::pair<Eigen::Vector3d, double>> waypoints_;
  bool have_waypoints_;
  int current_waypoint_idx_;
  Eigen::Vector3d target_pt_;
  double target_yaw_;
  ros::Time waypoint_start_time_;
  double waypoint_timeout_;
  double waypoint_tolerance_;  // a waypoint is reached within this distance (m)
  double stall_radius_;        // ... or within this one once the vehicle has stopped (m)
  double stall_time_;          // stopped for this long (s)
  ros::Time stopped_since_;    // zero while moving

  // Parameters
  double max_exploration_time_;
  Eigen::Vector3d exploration_center_;
  double exploration_width_;
  double exploration_height_;
  double exploration_altitude_;
  double line_spacing_;
  double replan_time_;
  bool enable_corner_smoothing_;

  // State strings for visualization
  std::vector<std::string> state_str_;
};

} // namespace exploration_manager

#endif