#include "exploration_manager/exploration_fsm.h"
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PositionCommand.h>

namespace exploration_manager {

void ExplorationFSM::init(ros::NodeHandle& nh) {
  nh_ = nh;
  
  // Load parameters
  nh.param("fsm/fsm_exec_dt", fsm_exec_dt_, 0.1);
  nh.param("fsm/replan_time", replan_time_, 0.05);
  nh.param("fsm/max_exploration_time", max_exploration_time_, 600.0); // 10 min default
  nh.param("fsm/frontier_search_interval", frontier_search_interval_, 2.0);
  
  // Initialize modules
  frontier_finder_.reset(new FrontierFinder);
  grid_map_.reset(new GridMap);
  
  // Initialize grid map
  grid_map_->initMap(nh);
  
  // Initialize frontier finder with grid map
  frontier_finder_->init(nh, grid_map_);
  
  // Store target for tracking
  target_pt_ = Eigen::Vector3d::Zero();
  
  // Initialize state
  state_ = EXPL_STATE::INIT;
  last_state_ = EXPL_STATE::INIT;
  have_odom_ = false;
  have_frontier_ = false;
  exploration_triggered_ = false;
  replan_count_ = 0;
  
  // Subscribers
  odom_sub_ = nh.subscribe("/mavros/local_position/odom", 10, 
                           &ExplorationFSM::odometryCallback, this);
  trigger_sub_ = nh.subscribe("/move_base_simple/goal", 10,
                              &ExplorationFSM::triggerCallback, this);
  finish_sub_ = nh.subscribe("/exploration/finish", 10,
                             &ExplorationFSM::finishCallback, this);
  
  // Publishers
  goal_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/drone_0_planning/pos_cmd", 10);
  state_pub_ = nh.advertise<std_msgs::String>("/exploration/state", 10);
  
  // Timer
  fsm_timer_ = nh.createTimer(ros::Duration(fsm_exec_dt_), 
                              &ExplorationFSM::fsmCallback, this);
  
  ROS_INFO("[ExplorationFSM] Initialized. Waiting for trigger...");
}

void ExplorationFSM::fsmCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    visualizeExplorationState();
    fsm_num = 0;
  }
  
  switch (state_) {
    case INIT:
      initState();
      break;
    case WAIT_TRIGGER:
      waitTriggerState();
      break;
    case SEARCH_FRONTIER:
      searchFrontierState();
      break;
    case PLAN_TO_FRONTIER:
      planToFrontierState();
      break;
    case EXEC_TRAJ:
      execTrajState();
      break;
    case FINISH:
      finishState();
      break;
  }
}

void ExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  string state_str[6] = {"INIT", "WAIT_TRIGGER", "SEARCH_FRONTIER", 
                         "PLAN_TO_FRONTIER", "EXEC_TRAJ", "FINISH"};
  int pre_s = int(state_);
  state_ = new_state;
  ROS_INFO("[ExplorationFSM] [%s]: %s -> %s", pos_call.c_str(), 
           state_str[pre_s].c_str(), state_str[int(new_state)].c_str());
}

void ExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_ = *msg;
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;
  
  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;
  
  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;
  
  have_odom_ = true;
}

void ExplorationFSM::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  if (!exploration_triggered_) {
    ROS_INFO("[ExplorationFSM] Exploration triggered!");
    exploration_triggered_ = true;
    exploration_start_time_ = ros::Time::now();
    
    if (state_ == WAIT_TRIGGER) {
      transitState(SEARCH_FRONTIER, "triggerCallback");
    }
  }
}

void ExplorationFSM::finishCallback(const std_msgs::EmptyConstPtr& msg) {
  ROS_WARN("[ExplorationFSM] Manual finish triggered!");
  transitState(FINISH, "finishCallback");
}

void ExplorationFSM::initState() {
  if (!have_odom_) {
    ROS_WARN_THROTTLE(2.0, "[ExplorationFSM] Waiting for odometry...");
    return;
  }
  
  ROS_INFO("[ExplorationFSM] Odometry received. Ready to explore.");
  transitState(WAIT_TRIGGER, "initState");
}

void ExplorationFSM::waitTriggerState() {
  if (exploration_triggered_) {
    transitState(SEARCH_FRONTIER, "waitTriggerState");
  }
}

void ExplorationFSM::searchFrontierState() {
  // Check if exploration time exceeded
  if ((ros::Time::now() - exploration_start_time_).toSec() > max_exploration_time_) {
    ROS_WARN("[ExplorationFSM] Max exploration time reached!");
    transitState(FINISH, "searchFrontierState");
    return;
  }
  
  // Search for frontiers
  ros::Time search_start = ros::Time::now();
  frontier_finder_->searchFrontiers();
  last_frontier_search_time_ = ros::Time::now();
  
  auto frontiers = frontier_finder_->getFrontiers();
  if (frontiers.empty()) {
    ROS_INFO("[ExplorationFSM] No frontiers found. Exploration complete!");
    transitState(FINISH, "searchFrontierState");
    return;
  }
  
  // Get best frontier
  current_frontier_ = frontier_finder_->getBestFrontier(odom_pos_, odom_vel_);
  have_frontier_ = true;
  
  // Visualize
  frontier_finder_->publishFrontiers();
  frontier_finder_->publishFrontierCells(current_frontier_.cells_);
  
  transitState(PLAN_TO_FRONTIER, "searchFrontierState");
}

void ExplorationFSM::planToFrontierState() {
  if (!have_frontier_) {
    ROS_ERROR("[ExplorationFSM] No frontier to plan to!");
    transitState(SEARCH_FRONTIER, "planToFrontierState");
    return;
  }
  
  // Select best viewpoint for the frontier
  if (current_frontier_.viewpoints_.empty()) {
    ROS_WARN("[ExplorationFSM] No valid viewpoints for frontier!");
    transitState(SEARCH_FRONTIER, "planToFrontierState");
    return;
  }
  
  // Use first viewpoint (could be improved with better selection)
  Viewpoint target_vp = current_frontier_.viewpoints_[0];
  target_pt_ = target_vp.pos_; // Store for later tracking
  
  ROS_INFO("[ExplorationFSM] Planning to frontier at (%.2f, %.2f, %.2f)", 
           target_pt_(0), target_pt_(1), target_pt_(2));
  
  // Publish target as PositionCommand for EGO planner/px4ctrl
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "odom";
  cmd.position.x = target_pt_(0);
  cmd.position.y = target_pt_(1);
  cmd.position.z = target_pt_(2);
  cmd.yaw = target_vp.yaw_;
  goal_pub_.publish(cmd);
  
  replan_count_ = 0;
  transitState(EXEC_TRAJ, "planToFrontierState");
}

void ExplorationFSM::execTrajState() {
  // Check if trajectory execution is complete
  double dist_to_goal = (odom_pos_ - target_pt_).norm();
  
  if (dist_to_goal < 0.5) {
    ROS_INFO("[ExplorationFSM] Reached frontier viewpoint!");
    
    // Mark this frontier region as explored (to avoid revisiting)
    frontier_finder_->markAsExplored(target_pt_);
    ROS_INFO("[ExplorationFSM] Marked region around (%.2f, %.2f, %.2f) as explored",
             target_pt_(0), target_pt_(1), target_pt_(2));
    
    // Wait and search for new frontiers
    ros::Duration(1.0).sleep(); // Give time for map to update
    transitState(SEARCH_FRONTIER, "execTrajState");
    return;
  }
  
  // Periodically search for new frontiers during execution
  if ((ros::Time::now() - last_frontier_search_time_).toSec() > frontier_search_interval_) {
    ROS_INFO("[ExplorationFSM] Periodic frontier search during execution");
    transitState(SEARCH_FRONTIER, "execTrajState");
    return;
  }
  
  // Note: Collision checking would be done by EGO planner FSM
  // This exploration FSM just manages high-level frontier selection
}

void ExplorationFSM::finishState() {
  ROS_INFO_THROTTLE(5.0, "[ExplorationFSM] Exploration finished. Hovering...");
  
  // Publish hover command
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "odom";
  cmd.position.x = odom_pos_(0);
  cmd.position.y = odom_pos_(1);
  cmd.position.z = odom_pos_(2);
  cmd.velocity.x = 0;
  cmd.velocity.y = 0;
  cmd.velocity.z = 0;
  cmd.acceleration.x = 0;
  cmd.acceleration.y = 0;
  cmd.acceleration.z = 0;
  cmd.yaw = 0;
  cmd.yaw_dot = 0;
  
  goal_pub_.publish(cmd);
}

void ExplorationFSM::visualizeExplorationState() {
  std_msgs::String state_msg;
  string state_str[6] = {"INIT", "WAIT_TRIGGER", "SEARCH_FRONTIER", 
                         "PLAN_TO_FRONTIER", "EXEC_TRAJ", "FINISH"};
  state_msg.data = state_str[int(state_)];
  state_pub_.publish(state_msg);
}

} // namespace exploration_manager
