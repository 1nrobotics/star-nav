#include "exploration_manager/waypoint_exploration.h"

namespace exploration_manager {

void WaypointExploration::init(ros::NodeHandle& nh) {
  nh_ = nh;

  // Load parameters
  nh.param("exploration/center_x", exploration_center_(0), 0.0);
  nh.param("exploration/center_y", exploration_center_(1), 0.0);
  nh.param("exploration/center_z", exploration_center_(2), 1.5);
  nh.param("exploration/width", exploration_width_, 9.0);
  nh.param("exploration/height", exploration_height_, 9.0);
  nh.param("exploration/altitude", exploration_altitude_, 1.0);
  nh.param("exploration/line_spacing", line_spacing_, 3.0);
  nh.param("exploration/enable_corner_smoothing", enable_corner_smoothing_, false);
  nh.param("fsm/waypoint_timeout", waypoint_timeout_, 30.0);
  nh.param("fsm/max_exploration_time", max_exploration_time_, 600.0);
  nh.param("fsm/replan_time", replan_time_, 0.05);

  ROS_INFO("[WaypointExploration] Loaded parameters:");
  ROS_INFO("  exploration/center: [%.2f, %.2f, %.2f]", exploration_center_(0), exploration_center_(1), exploration_center_(2));
  ROS_INFO("  exploration/size: %.2f x %.2f m", exploration_width_, exploration_height_);
  ROS_INFO("  exploration/altitude: %.2f m", exploration_altitude_);
  ROS_INFO("  exploration/line_spacing: %.2f m", line_spacing_);
  ROS_INFO("  exploration/enable_corner_smoothing: %s", enable_corner_smoothing_ ? "true" : "false");
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
                     "Distance: %.3f m (H:%.3f, V:%.3f) | Tolerance: 0.5 m",
                     current_waypoint_idx_,
                     odom_pos_(0), odom_pos_(1), odom_pos_(2),
                     target_pt_(0), target_pt_(1), target_pt_(2),
                     dist_to_goal, horizontal_dist, fabs(dz));

  // Check if reached waypoint
  if (dist_to_goal < 0.5) {
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
  
  // Add lead-in waypoints for smooth approach from origin (0,0)
  // (0,0) → (max_x - 1.5, max_y) → (max_x - 0.5, max_y) → (max_x, max_y - 0.5)
  const double offset1 = line_spacing_ * 0.5;   // 1.5m for 3m spacing
  const double offset2 = line_spacing_ / 6.0;   // 0.5m for 3m spacing
  
  Eigen::Vector3d leadin1(max_x - offset1, max_y, exploration_altitude_);
  raw_waypoints.push_back(leadin1);
  ROS_INFO("  Lead-in 1: [%.2f, %.2f, %.2f]", leadin1(0), leadin1(1), leadin1(2));
  
  Eigen::Vector3d leadin2(max_x - offset2, max_y, exploration_altitude_);
  raw_waypoints.push_back(leadin2);
  ROS_INFO("  Lead-in 2: [%.2f, %.2f, %.2f]", leadin2(0), leadin2(1), leadin2(2));
  
  Eigen::Vector3d leadin3(max_x, max_y - offset2, exploration_altitude_);
  raw_waypoints.push_back(leadin3);
  ROS_INFO("  Lead-in 3: [%.2f, %.2f, %.2f]", leadin3(0), leadin3(1), leadin3(2));
  
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

  // Optionally smooth 90° corners by inserting 4 intermediate waypoints
  if (enable_corner_smoothing_) {
    ROS_INFO("[WaypointExploration] Smoothing large corners with 4 intermediate waypoints...");
    
    const double STEP = line_spacing_ * 0.25;  // Quarter of line spacing per step
    
    for (size_t i = 0; i < raw_waypoints.size(); ++i) {
      // Check if current waypoint is a 90° corner that should be smoothed
      bool is_corner = false;
      
      if (i >= 1 && i < raw_waypoints.size() - 1) {
        Eigen::Vector3d prev = raw_waypoints[i-1];
        Eigen::Vector3d curr = raw_waypoints[i];
        Eigen::Vector3d next = raw_waypoints[i+1];
        
        Eigen::Vector3d dir1_3d = curr - prev;
        Eigen::Vector3d dir2_3d = next - curr;
        
        if (dir1_3d.norm() > 0.01 && dir2_3d.norm() > 0.01) {
          dir1_3d.normalize();
          dir2_3d.normalize();
          
          double dot = dir1_3d.dot(dir2_3d);
          double angle = std::acos(std::max(-1.0, std::min(1.0, dot)));
          
          // If ~90° turn (between 60° and 120°)
          if (angle > M_PI / 3.0 && angle < 2.0 * M_PI / 3.0) {
            is_corner = true;
            ROS_INFO("    Large corner at waypoint %lu [%.2f, %.2f], replacing with 4 intermediate points", 
                     i, curr(0), curr(1));
            
            // Insert 4 intermediate waypoints to create smooth arc
            Eigen::Vector3d int1 = curr - dir1_3d * STEP;  // Step back on incoming direction
            waypoints_.push_back(std::make_pair(int1, 0.0));
            ROS_INFO("      Int1: [%.2f, %.2f, %.2f]", int1(0), int1(1), int1(2));
            
            Eigen::Vector3d int2 = curr + (dir2_3d - dir1_3d).normalized() * STEP;  // Diagonal
            waypoints_.push_back(std::make_pair(int2, 0.0));
            ROS_INFO("      Int2: [%.2f, %.2f, %.2f]", int2(0), int2(1), int2(2));
            
            Eigen::Vector3d int3 = curr + dir2_3d * STEP;  // Step forward on outgoing direction
            waypoints_.push_back(std::make_pair(int3, 0.0));
            ROS_INFO("      Int3: [%.2f, %.2f, %.2f]", int3(0), int3(1), int3(2));
            
            Eigen::Vector3d int4 = curr + dir2_3d * (2.0 * STEP);  // Further on outgoing direction
            waypoints_.push_back(std::make_pair(int4, 0.0));
            ROS_INFO("      Int4: [%.2f, %.2f, %.2f]", int4(0), int4(1), int4(2));
          }
        }
      }
      
      // Add current waypoint only if it's not a corner (corners are replaced by intermediate points)
      if (!is_corner) {
        waypoints_.push_back(std::make_pair(raw_waypoints[i], 0.0));
      }
    }
    
    ROS_INFO("[WaypointExploration] Generated %lu waypoints with smooth 4-point corners", waypoints_.size());
  } else {
    // Corner smoothing disabled - use raw waypoints directly
    ROS_INFO("[WaypointExploration] Corner smoothing disabled, using raw waypoints");
    for (size_t i = 0; i < raw_waypoints.size(); ++i) {
      waypoints_.push_back(std::make_pair(raw_waypoints[i], 0.0));
    }
    ROS_INFO("[WaypointExploration] Generated %lu waypoints (no smoothing)", waypoints_.size());
  }
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