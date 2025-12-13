#ifndef EXPLORATION_FSM_H_
#define EXPLORATION_FSM_H_

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

#include "exploration_manager/frontier_finder.h"
#include <plan_manage/planner_manager.h>

namespace exploration_manager {

enum EXPL_STATE {
  INIT,
  WAIT_TRIGGER,
  SEARCH_FRONTIER,
  PLAN_TO_FRONTIER,
  EXEC_TRAJ,
  FINISH
};

class ExplorationFSM {
public:
  ExplorationFSM() {}
  ~ExplorationFSM() {}

  void init(ros::NodeHandle& nh);

private:
  // State machine callbacks
  void fsmCallback(const ros::TimerEvent& e);
  void transitState(EXPL_STATE new_state, string pos_call);
  
  // ROS callbacks
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void finishCallback(const std_msgs::EmptyConstPtr& msg);
  
  // Exploration logic
  bool findAndPlanToFrontier();
  bool replanFromCurrentState();
  void visualizeExplorationState();
  
  // State handlers
  void initState();
  void waitTriggerState();
  void searchFrontierState();
  void planToFrontierState();
  void execTrajState();
  void finishState();

  // Modules
  shared_ptr<FrontierFinder> frontier_finder_;
  shared_ptr<GridMap> grid_map_; // Shared with EGO planner
  
  // ROS
  ros::NodeHandle nh_;
  ros::Timer fsm_timer_;
  ros::Subscriber odom_sub_, trigger_sub_, finish_sub_;
  ros::Publisher goal_pub_, state_pub_;
  
  // State
  EXPL_STATE state_;
  EXPL_STATE last_state_;
  
  // Odometry
  nav_msgs::Odometry odom_;
  Eigen::Vector3d odom_pos_, odom_vel_;
  Eigen::Quaterniond odom_orient_;
  bool have_odom_;
  
  // Exploration state
  Frontier current_frontier_;
  Eigen::Vector3d target_pt_; // Current target point
  bool have_frontier_;
  bool exploration_triggered_;
  int replan_count_;
  
  // Parameters
  double fsm_exec_dt_;
  double replan_time_;
  double max_exploration_time_;
  double frontier_search_interval_;
  
  ros::Time last_frontier_search_time_;
  ros::Time exploration_start_time_;
};

} // namespace exploration_manager

#endif // EXPLORATION_FSM_H_
