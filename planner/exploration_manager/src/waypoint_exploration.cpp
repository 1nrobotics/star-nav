#include "exploration_manager/waypoint_exploration.h"

namespace exploration_manager {

void WaypointExploration::init(ros::NodeHandle& nh, shared_ptr<GridMap>& grid_map) {
  nh_ = nh;
  grid_map_ = grid_map;

  // Load parameters
  nh.param("exploration/center_x", exploration_center_(0), 0.0);
  nh.param("exploration/center_y", exploration_center_(1), 0.0);
  nh.param("exploration/center_z", exploration_center_(2), 1.5);
  nh.param("exploration/width", exploration_width_, 10.0);
  nh.param("exploration/height", exploration_height_, 10.0);
  nh.param("exploration/altitude", exploration_altitude_, 1.5);
  nh.param("exploration/line_spacing", line_spacing_, 0.5);
  nh.param("fsm/waypoint_timeout", waypoint_timeout_, 30.0);
  nh.param("fsm/max_exploration_time", max_exploration_time_, 600.0);
  nh.param("fsm/replan_time", replan_time_, 0.05);

  // Initialize state
  state_ = INIT;
  exploration_triggered_ = false;
  have_odom_ = false;
  have_waypoints_ = false;
  current_waypoint_idx_ = 0;

  // State strings
  state_str_ = {"INIT", "WAIT_TRIGGER", "GENERATE_WAYPOINTS", "PLAN_TO_WAYPOINT", "EXEC_WAYPOINT", "FINISH"};

  // Publishers
  goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  state_pub_ = nh.advertise<std_msgs::String>("/exploration/state", 10);

  // Subscribers
  odom_sub_ = nh.subscribe("/mavros/local_position/odom", 10, &WaypointExploration::odometryCallback, this);
  trigger_sub_ = nh.subscribe("/move_base_simple/goal", 10, &WaypointExploration::triggerCallback, this);
  finish_sub_ = nh.subscribe("/exploration/finish", 10, &WaypointExploration::finishCallback, this);

  // FSM timer
  fsm_timer_ = nh.createTimer(ros::Duration(replan_time_), &WaypointExploration::fsmCallback, this);

  ROS_INFO("[WaypointExploration] Initialized");
  ROS_INFO("  - Exploration area: %.1fx%.1f m at [%.1f, %.1f, %.1f]",
           exploration_width_, exploration_height_,
           exploration_center_(0), exploration_center_(1), exploration_center_(2));
  ROS_INFO("  - Line spacing: %.2f m, Timeout: %.1f s", line_spacing_, waypoint_timeout_);
}

void WaypointExploration::fsmCallback(const ros::TimerEvent& e) {
  switch (state_) {
    case INIT:
      initState();
      break;
    case WAIT_TRIGGER:
      waitTriggerState();
      break;
    case GENERATE_WAYPOINTS:
      generateWaypointsState();
      break;
    case PLAN_TO_WAYPOINT:
      planToWaypointState();
      break;
    case EXEC_WAYPOINT:
      execWaypointState();
      break;
    case FINISH:
      finishState();
      break;
  }

  visualizeState();
}

void WaypointExploration::initState() {
  if (!have_odom_) {
    ROS_WARN("[WaypointExploration] Waiting for odometry...");
    return;
  }

  transitState(WAIT_TRIGGER, "initState");
}

void WaypointExploration::waitTriggerState() {
  // Wait for exploration trigger
  if (exploration_triggered_) {
    ROS_INFO("[WaypointExploration] Exploration triggered!");
    exploration_start_time_ = ros::Time::now();
    transitState(GENERATE_WAYPOINTS, "waitTriggerState");
  }
}

void WaypointExploration::generateWaypointsState() {
  // Check timeout
  if ((ros::Time::now() - exploration_start_time_).toSec() > max_exploration_time_) {
    ROS_WARN("[WaypointExploration] Max exploration time reached!");
    transitState(FINISH, "generateWaypointsState");
    return;
  }

  // Generate lawnmower waypoints
  generateLawnmowerWaypoints();

  if (waypoints_.empty()) {
    ROS_WARN("[WaypointExploration] No valid waypoints generated!");
    transitState(FINISH, "generateWaypointsState");
    return;
  }

  have_waypoints_ = true;
  current_waypoint_idx_ = 0;

  ROS_INFO("[WaypointExploration] Generated %lu waypoints, starting with waypoint 0", waypoints_.size());
  transitState(PLAN_TO_WAYPOINT, "generateWaypointsState");
}

void WaypointExploration::planToWaypointState() {
  if (!have_waypoints_ || current_waypoint_idx_ >= (int)waypoints_.size()) {
    ROS_INFO("[WaypointExploration] All waypoints completed!");
    transitState(FINISH, "planToWaypointState");
    return;
  }

  // Get next waypoint
  target_pt_ = waypoints_[current_waypoint_idx_];

  // Publish goal
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "world";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = target_pt_(0);
  goal.pose.position.y = target_pt_(1);
  goal.pose.position.z = target_pt_(2);
  goal.pose.orientation.w = 1.0;

  goal_pub_.publish(goal);
  waypoint_start_time_ = ros::Time::now();

  ROS_INFO("[WaypointExploration] Planning to waypoint %d: [%.2f, %.2f, %.2f]",
           current_waypoint_idx_, target_pt_(0), target_pt_(1), target_pt_(2));

  transitState(EXEC_WAYPOINT, "planToWaypointState");
}

void WaypointExploration::execWaypointState() {
  double dist_to_goal = (odom_pos_ - target_pt_).norm();

  // Check if reached waypoint
  if (dist_to_goal < 0.5) {  // 0.5m tolerance
    ROS_INFO("[WaypointExploration] Reached waypoint %d!", current_waypoint_idx_);
    current_waypoint_idx_++;
    transitState(PLAN_TO_WAYPOINT, "execWaypointState");
    return;
  }

  // Check timeout
  double elapsed = (ros::Time::now() - waypoint_start_time_).toSec();
  if (elapsed > waypoint_timeout_) {
    ROS_WARN("[WaypointExploration] Waypoint %d timeout (%.1fs)! Moving to next waypoint.",
             current_waypoint_idx_, elapsed);
    current_waypoint_idx_++;
    transitState(PLAN_TO_WAYPOINT, "execWaypointState");
    return;
  }
}

void WaypointExploration::finishState() {
  // Reset state
  exploration_triggered_ = false;
  have_waypoints_ = false;
  current_waypoint_idx_ = 0;
  waypoints_.clear();

  ROS_INFO("[WaypointExploration] Exploration finished, returning to WAIT_TRIGGER");
  transitState(WAIT_TRIGGER, "finishState");
}

void WaypointExploration::generateLawnmowerWaypoints() {
  waypoints_.clear();

  // Calculate rectangle bounds
  double half_width = exploration_width_ / 2.0;
  double half_height = exploration_height_ / 2.0;

  double min_x = exploration_center_(0) - half_width;
  double max_x = exploration_center_(0) + half_width;
  double min_y = exploration_center_(1) - half_height;
  double max_y = exploration_center_(1) + half_height;

  // Generate lawnmower pattern
  bool move_right = true;
  double current_y = min_y;

  while (current_y <= max_y) {
    if (move_right) {
      // Move from left to right
      for (double x = min_x; x <= max_x; x += line_spacing_) {
        Eigen::Vector3d wp(x, current_y, exploration_altitude_);
        if (isValidWaypoint(wp)) {
          waypoints_.push_back(wp);
        }
      }
    } else {
      // Move from right to left
      for (double x = max_x; x >= min_x; x -= line_spacing_) {
        Eigen::Vector3d wp(x, current_y, exploration_altitude_);
        if (isValidWaypoint(wp)) {
          waypoints_.push_back(wp);
        }
      }
    }

    // Move to next row
    current_y += line_spacing_;
    move_right = !move_right;  // Alternate direction
  }

  ROS_INFO("[WaypointExploration] Generated %lu waypoints in lawnmower pattern", waypoints_.size());
}

bool WaypointExploration::isValidWaypoint(const Eigen::Vector3d& pos) {
  // Check if position is in free space
  if (grid_map_->getInflateOccupancy(pos) != 0) {
    return false; // Occupied or unknown
  }

  // Check if within map bounds
  Eigen::Vector3d map_min, map_max;
  grid_map_->getRegion(map_min, map_max);

  if (pos(0) < map_min(0) || pos(0) > map_max(0) ||
      pos(1) < map_min(1) || pos(1) > map_max(1) ||
      pos(2) < map_min(2) || pos(2) > map_max(2)) {
    return false;
  }

  return true;
}

void WaypointExploration::transitState(EXPL_STATE new_state, string pos_call) {
  string state_str = state_str_[int(state_)];
  state_ = new_state;
  ROS_INFO("[WaypointExploration] %s -> %s (%s)", state_str.c_str(), state_str_[int(new_state)].c_str(), pos_call.c_str());
}

void WaypointExploration::visualizeState() {
  std_msgs::String state_msg;
  state_msg.data = state_str_[int(state_)];
  state_pub_.publish(state_msg);
}

void WaypointExploration::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
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

void WaypointExploration::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  if (!exploration_triggered_) {
    exploration_triggered_ = true;
    ROS_INFO("[WaypointExploration] Exploration triggered via topic");
  }
}

void WaypointExploration::finishCallback(const std_msgs::EmptyConstPtr& msg) {
  ROS_INFO("[WaypointExploration] Manual finish command received");
  transitState(FINISH, "finishCallback");
}

void WaypointExploration::startExploration() {
  exploration_triggered_ = true;
  exploration_start_time_ = ros::Time::now();
}

void WaypointExploration::stopExploration() {
  transitState(FINISH, "stopExploration");
}

} // namespace exploration_manager