#include "exploration_manager/waypoint_exploration.h"

namespace exploration_manager {

void WaypointExploration::init(ros::NodeHandle& nh) {
  nh_ = nh;

  // Load parameters
  nh.param("exploration/center_x", exploration_center_(0), 0.0);
  nh.param("exploration/center_y", exploration_center_(1), 0.0);
  nh.param("exploration/center_z", exploration_center_(2), 1.5);
  nh.param("exploration/width", exploration_width_, 10.0);
  nh.param("exploration/height", exploration_height_, 10.0);
  nh.param("exploration/altitude", exploration_altitude_, 1.0);
  nh.param("exploration/line_spacing", line_spacing_, 0.5);
  nh.param("fsm/waypoint_timeout", waypoint_timeout_, 30.0);
  nh.param("fsm/max_exploration_time", max_exploration_time_, 600.0);
  nh.param("fsm/replan_time", replan_time_, 0.05);

  ROS_INFO("[WaypointExploration] Loaded parameters:");
  ROS_INFO("  exploration/center: [%.2f, %.2f, %.2f]", exploration_center_(0), exploration_center_(1), exploration_center_(2));
  ROS_INFO("  exploration/size: %.2f x %.2f m", exploration_width_, exploration_height_);
  ROS_INFO("  exploration/altitude: %.2f m", exploration_altitude_);
  ROS_INFO("  exploration/line_spacing: %.2f m", line_spacing_);
  ROS_INFO("  fsm/waypoint_timeout: %.2f s", waypoint_timeout_);
  ROS_INFO("  fsm/max_exploration_time: %.2f s", max_exploration_time_);
  ROS_INFO("  fsm/replan_time: %.2f s", replan_time_);

  // Initialize state
  state_ = INIT;
  exploration_triggered_ = false;
  have_odom_ = false;
  have_waypoints_ = false;
  current_waypoint_idx_ = 0;

  // State strings
  state_str_.assign({"INIT", "WAIT_TRIGGER", "GENERATE_WAYPOINTS", "PLAN_TO_WAYPOINT", "EXEC_WAYPOINT", "FINISH"});

  // Publishers
  goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  state_pub_ = nh.advertise<std_msgs::String>("/exploration/state", 10);
  params_pub_ = nh.advertise<std_msgs::String>("/exploration/parameters", 1, true); // latched

  // Publish parameters for GCS
  std_msgs::String params_msg;
  char buffer[256];
  sprintf(buffer, "Center: (%.1f, %.1f) Size: %.1fx%.1f Spacing: %.1f",
          exploration_center_(0), exploration_center_(1), exploration_width_, exploration_height_, line_spacing_);
  params_msg.data = buffer;
  params_pub_.publish(params_msg);

  // Subscribers
  odom_sub_ = nh.subscribe("/mavros/local_position/odom", 10, &WaypointExploration::odometryCallback, this);
  trigger_sub_ = nh.subscribe("/exploration/trigger", 10, &WaypointExploration::triggerCallback, this);
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
    ROS_WARN_THROTTLE(5.0, "[WaypointExploration] Waiting for odometry...");
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

  // Get next waypoint (position)
  auto& waypoint = waypoints_[current_waypoint_idx_];
  target_pt_ = waypoint.first;
  
  // Set heading to 0 (facing positive X direction)
  target_yaw_ = 0.0;

  // Publish goal with position and orientation
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "world";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = target_pt_(0);
  goal.pose.position.y = target_pt_(1);
  goal.pose.position.z = target_pt_(2);
  
  // Set orientation from yaw angle
  tf::Quaternion q;
  q.setRPY(0, 0, target_yaw_);
  goal.pose.orientation.x = q.x();
  goal.pose.orientation.y = q.y();
  goal.pose.orientation.z = q.z();
  goal.pose.orientation.w = q.w();

  goal_pub_.publish(goal);
  waypoint_start_time_ = ros::Time::now();

  ROS_INFO("[WaypointExploration] Planning to waypoint %d: [%.2f, %.2f, %.2f] yaw=%.1f deg",
           current_waypoint_idx_, target_pt_(0), target_pt_(1), target_pt_(2), 
           target_yaw_ * 180.0 / M_PI);

  transitState(EXEC_WAYPOINT, "planToWaypointState");
}

void WaypointExploration::execWaypointState() {
  double dist_to_goal = (odom_pos_ - target_pt_).norm();
  
  // Detailed distance analysis
  double dx = odom_pos_(0) - target_pt_(0);
  double dy = odom_pos_(1) - target_pt_(1);
  double dz = odom_pos_(2) - target_pt_(2);
  double horizontal_dist = sqrt(dx*dx + dy*dy);

  // Log progress towards waypoint (throttled to 5Hz for debug)
  ROS_DEBUG_THROTTLE(5.0, "[WaypointExploration] Waypoint %d progress: "
                     "Current [%.2f, %.2f, %.2f] -> Target [%.2f, %.2f, %.2f] | "
                     "Distance: %.3f m (H:%.3f, V:%.3f) | Tolerance: 0.7 m",
                     current_waypoint_idx_,
                     odom_pos_(0), odom_pos_(1), odom_pos_(2),
                     target_pt_(0), target_pt_(1), target_pt_(2),
                     dist_to_goal, horizontal_dist, fabs(dz));

  // Check if reached waypoint (increased tolerance for simulation)
  if (dist_to_goal < 0.7) {  // Increased from 0.5m to 0.7m for simulation
    ROS_INFO("[WaypointExploration] SUCCESS: Reached waypoint %d! Final distance: %.3f m (H:%.3f, V:%.3f)",
             current_waypoint_idx_, dist_to_goal, horizontal_dist, fabs(dz));
    current_waypoint_idx_++;
    transitState(PLAN_TO_WAYPOINT, "execWaypointState");
    return;
  }

  // Check timeout
  double elapsed = (ros::Time::now() - waypoint_start_time_).toSec();
  if (elapsed > waypoint_timeout_) {
    ROS_WARN("[WaypointExploration] TIMEOUT: Waypoint %d timeout (%.1fs)! Distance: %.3f m (H:%.3f, V:%.3f). Moving to next waypoint.",
             current_waypoint_idx_, elapsed, dist_to_goal, horizontal_dist, fabs(dz));
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

  ROS_INFO("[WaypointExploration] Generating lawnmower waypoints with 45° corner smoothing:");
  ROS_INFO("  Coordinate system: X=forward/backward, Y=left/right");
  ROS_INFO("  Center: [%.2f, %.2f, %.2f]", exploration_center_(0), exploration_center_(1), exploration_center_(2));
  ROS_INFO("  Size: %.2f x %.2f m", exploration_width_, exploration_height_);

  // Calculate rectangle bounds
  double half_width = exploration_width_ / 2.0;
  double half_height = exploration_height_ / 2.0;

  double min_x = exploration_center_(0) - half_width;  // backward
  double max_x = exploration_center_(0) + half_width;  // forward
  double min_y = exploration_center_(1) - half_height; // right
  double max_y = exploration_center_(1) + half_height; // left

  ROS_INFO("  Bounds: forward[%.2f, %.2f], left[%.2f, %.2f]", min_x, max_x, min_y, max_y);

  // Generate original lawnmower pattern first
  vector<Eigen::Vector3d> raw_waypoints;
  bool scan_left_to_right = true;
  double current_x = max_x;
  int row_count = 0;

  while (current_x >= min_x) {
    ROS_INFO("  Row %d: x=%.2f, direction=%s", 
             row_count, current_x, scan_left_to_right ? "left-to-right" : "right-to-left");
    
    if (scan_left_to_right) {
      // Scan from left to right
      Eigen::Vector3d start_wp(current_x, max_y, exploration_altitude_);
      Eigen::Vector3d end_wp(current_x, min_y, exploration_altitude_);
      
      raw_waypoints.push_back(start_wp);
      ROS_INFO("    Start: [%.2f, %.2f, %.2f]", start_wp(0), start_wp(1), start_wp(2));
      
      raw_waypoints.push_back(end_wp);
      ROS_INFO("    End: [%.2f, %.2f, %.2f]", end_wp(0), end_wp(1), end_wp(2));
    } else {
      // Scan from right to left
      Eigen::Vector3d start_wp(current_x, min_y, exploration_altitude_);
      Eigen::Vector3d end_wp(current_x, max_y, exploration_altitude_);
      
      raw_waypoints.push_back(start_wp);
      ROS_INFO("    Start: [%.2f, %.2f, %.2f]", start_wp(0), start_wp(1), start_wp(2));
      
      raw_waypoints.push_back(end_wp);
      ROS_INFO("    End: [%.2f, %.2f, %.2f]", end_wp(0), end_wp(1), end_wp(2));
    }

    current_x -= line_spacing_;
    scan_left_to_right = !scan_left_to_right;
    row_count++;
    
    if (row_count > 100) {
      ROS_ERROR("[WaypointExploration] Too many rows generated, breaking!");
      break;
    }
  }

  // Add return home waypoint
  raw_waypoints.push_back(Eigen::Vector3d(exploration_center_(0), exploration_center_(1), exploration_altitude_));
  ROS_INFO("  Added return home: [%.2f, %.2f, %.2f]", exploration_center_(0), exploration_center_(1), exploration_altitude_);

  // Now smooth 90° corners by inserting 4 intermediate waypoints
  ROS_INFO("[WaypointExploration] Smoothing 90° corners with 4 intermediate waypoints...");
  
  const double STEP = line_spacing_ * 0.25;  // Quarter of line spacing per step
  
  for (size_t i = 0; i < raw_waypoints.size(); ++i) {
    if (i == 0) {
      // First waypoint - add as-is
      waypoints_.push_back(std::make_pair(raw_waypoints[i], 0.0));
      continue;
    }
    
    if (i >= 2) {
      // Check if there's a 90° turn at previous waypoint
      Eigen::Vector3d prev_prev = raw_waypoints[i-2];
      Eigen::Vector3d prev = raw_waypoints[i-1];
      Eigen::Vector3d curr = raw_waypoints[i];
      
      Eigen::Vector2d dir1(prev(0) - prev_prev(0), prev(1) - prev_prev(1));
      Eigen::Vector2d dir2(curr(0) - prev(0), curr(1) - prev(1));
      
      if (dir1.norm() > 0.01 && dir2.norm() > 0.01) {
        dir1.normalize();
        dir2.normalize();
        
        double dot = dir1.dot(dir2);
        double angle = std::acos(std::max(-1.0, std::min(1.0, dot)));
        
        // If ~90° turn (between 60° and 120°)
        if (angle > M_PI / 3.0 && angle < 2.0 * M_PI / 3.0) {
          ROS_INFO("    90° turn at waypoint %lu [%.2f, %.2f], inserting 4 intermediate points", 
                   i-1, prev(0), prev(1));
          
          // Insert 4 intermediate waypoints to create smooth arc
          // Example: (10,-10) to (8,-10) becomes:
          // (10,-9.5), (9.5,-10), (8.5,-10), (8,-9.5)
          
          Eigen::Vector3d int1 = prev - dir1 * STEP;  // Step back on incoming direction
          waypoints_.push_back(std::make_pair(int1, 0.0));
          ROS_INFO("      Int1: [%.2f, %.2f, %.2f]", int1(0), int1(1), int1(2));
          
          Eigen::Vector3d int2 = prev + (dir2 - dir1).normalized() * STEP;  // Diagonal
          waypoints_.push_back(std::make_pair(int2, 0.0));
          ROS_INFO("      Int2: [%.2f, %.2f, %.2f]", int2(0), int2(1), int2(2));
          
          Eigen::Vector3d int3 = prev + dir2 * STEP;  // Step forward on outgoing direction
          waypoints_.push_back(std::make_pair(int3, 0.0));
          ROS_INFO("      Int3: [%.2f, %.2f, %.2f]", int3(0), int3(1), int3(2));
          
          Eigen::Vector3d int4 = prev + dir2 * (2.0 * STEP);  // Further on outgoing direction
          waypoints_.push_back(std::make_pair(int4, 0.0));
          ROS_INFO("      Int4: [%.2f, %.2f, %.2f]", int4(0), int4(1), int4(2));
        }
      }
    }
    
    // Add current waypoint
    waypoints_.push_back(std::make_pair(raw_waypoints[i], 0.0));
  }

  ROS_INFO("[WaypointExploration] Generated %lu waypoints with smooth 4-point corners", waypoints_.size());
}

bool WaypointExploration::isValidWaypoint(const Eigen::Vector3d& pos) {
  // Simplified: always consider waypoints valid since we don't use grid map validation
  return true;
}

void WaypointExploration::transitState(EXPL_STATE new_state, std::string pos_call) {
  std::string state_str = state_str_[int(state_)];
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