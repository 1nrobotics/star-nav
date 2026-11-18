#include "exploration_manager/frontier_exploration.h"

#include <algorithm>
#include <cmath>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

namespace exploration_manager {

void FrontierExploration::init(ros::NodeHandle& nh) {
  double cx, cy, width, height;
  nh.param("exploration/center_x", cx, 0.0);
  nh.param("exploration/center_y", cy, 0.0);
  nh.param("exploration/width", width, 9.0);
  nh.param("exploration/height", height, 9.0);
  nh.param("exploration/altitude", altitude_, 1.0);
  nh.param("fsm/max_exploration_time", max_exploration_time_, 600.0);
  nh.param("fsm/waypoint_tolerance", goal_tolerance_, 0.5);
  area_min_ = Eigen::Vector2d(cx - width / 2.0, cy - height / 2.0);
  area_max_ = Eigen::Vector2d(cx + width / 2.0, cy + height / 2.0);

  FrontierMap::Params mp;
  nh.param("frontier/resolution", mp.resolution, 0.1);
  nh.param("frontier/inflation", mp.inflation, 0.3);
  nh.param("frontier/min_cells", mp.min_frontier_cells, 8);
  nh.param("frontier/max_cluster_size", mp.max_cluster_size, 3.0);
  nh.param("frontier/escape_radius", mp.escape_radius, 1.0);
  nh.param("frontier/goal_keep_radius", goal_keep_radius_, 0.6);
  nh.param("frontier/reach_radius", reach_radius_, 1.0);
  nh.param("frontier/min_goal_distance", min_goal_distance_, 2.0);
  nh.param("frontier/back_off", back_off_, 2.0);
  nh.param("frontier/min_goal_interval", min_goal_interval_, 2.0);
  nh.param("frontier/return_step", return_step_, 3.0);
  nh.param("frontier/switch_margin", switch_margin_, 2.0);
  nh.param("frontier/turn_weight", turn_weight_, 1.0);
  nh.param("frontier/size_weight", size_weight_, 0.5);
  nh.param("frontier/stuck_time", stuck_time_, 10.0);
  nh.param("frontier/blocked_radius", blocked_radius_, 1.0);
  nh.param("frontier/finish_confirm", finish_confirm_, 8);
  map_.init(area_min_, area_max_, mp);

  ROS_INFO("[FrontierExploration] area [%.1f, %.1f] x [%.1f, %.1f] at %.2f m, inflation %.2f m",
           area_min_(0), area_max_(0), area_min_(1), area_max_(1), altitude_, mp.inflation);

  state_names_ = {"INIT", "WAIT_TRIGGER", "EXPLORE", "RETURN_HOME", "FINISH"};

  goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  state_pub_ = nh.advertise<std_msgs::String>("/exploration/state", 10);
  coverage_pub_ = nh.advertise<std_msgs::Float32>("/exploration/coverage", 10);
  map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/exploration/map", 1, true);
  frontier_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/exploration/frontiers", 1, true);

  slice_sub_ = nh.subscribe("slice", 2, &FrontierExploration::sliceCallback, this);
  odom_sub_ =
      nh.subscribe("/mavros/local_position/odom", 10, &FrontierExploration::odometryCallback, this);
  trigger_sub_ =
      nh.subscribe("/exploration/trigger", 10, &FrontierExploration::triggerCallback, this);
  finish_sub_ = nh.subscribe("/exploration/finish", 10, &FrontierExploration::finishCallback, this);

  fsm_timer_ = nh.createTimer(ros::Duration(0.25), &FrontierExploration::fsmCallback, this);
}

void FrontierExploration::fsmCallback(const ros::TimerEvent& /*e*/) {
  switch (state_) {
    case INIT:
      if (have_odom_)
        transit(WAIT_TRIGGER, "odometry received");
      else
        ROS_WARN_THROTTLE(5.0, "[FrontierExploration] Waiting for odometry...");
      break;
    case WAIT_TRIGGER:
      if (triggered_) {
        triggered_ = false;
        home_ = odom_pos_.head<2>();
        start_time_ = ros::Time::now();
        have_goal_ = false;
        last_goal_time_ = ros::Time(0);
        blocked_.clear();
        route_.clear();
        no_frontier_cycles_ = 0;
        goals_sent_ = 0;
        transit(EXPLORE, "triggered");
      }
      break;
    case EXPLORE:
      exploreState();
      break;
    case RETURN_HOME:
      returnHomeState();
      break;
    case FINISH:
      finishState();
      break;
  }
  publishState();
}

void FrontierExploration::exploreState() {
  if ((ros::Time::now() - start_time_).toSec() > max_exploration_time_) {
    ROS_WARN("[FrontierExploration] Max exploration time reached, returning to the start");
    transit(RETURN_HOME, "time limit");
    return;
  }
  if (!have_slice_) {
    ROS_WARN_THROTTLE(5.0,
                      "[FrontierExploration] No map slice yet on %s (grid_map/slice_max_z set?)",
                      slice_sub_.getTopic().c_str());
    return;
  }

  const Eigen::Vector2d robot = odom_pos_.head<2>();
  map_.update(robot, blocked_, blocked_radius_);
  if (!map_.robotInFreeSpace())
    ROS_WARN_THROTTLE(5.0,
                      "[FrontierExploration] No observed free space around [%.2f, %.2f], waiting "
                      "(the map needs ray casting: a depth image, or a point cloud with "
                      "grid_map/enable_pointcloud_raycasting)",
                      robot(0), robot(1));

  if (have_goal_) {
    double d = map_.distance(goal_);
    if (d == FrontierMap::kUnreachable)
      d = (goal_ - robot).norm();
    // A frontier goal is reached within reach_radius: the camera has looked
    // at it on the way, and nearer it would only look past it.
    const double reach = backing_off_ ? goal_tolerance_ : std::max(goal_tolerance_, reach_radius_);
    if ((goal_ - robot).norm() < reach) {
      have_goal_ = false;
      last_goal_time_ = ros::Time(0);  // pick the next goal now
    } else if (!backing_off_ && !map_.nearFrontier(goal_, goal_keep_radius_)) {
      have_goal_ = false;  // its frontier has been observed on the way
    } else if (!makingProgress(d)) {
      ROS_WARN(
          "[FrontierExploration] No progress towards [%.2f, %.2f] for %.0f s (%.2f m away), "
          "blocking it",
          goal_(0), goal_(1), stuck_time_, (goal_ - robot).norm());
      last_goal_time_ = ros::Time(0);
      blocked_.push_back(goal_);
      have_goal_ = false;
      map_.update(robot, blocked_, blocked_radius_);
    }
  }

  if (!selectGoal()) {
    // Without observed free space around the robot (map not there yet) no
    // frontier can be reached: wait instead of finishing.
    if (map_.robotInFreeSpace() && ++no_frontier_cycles_ >= finish_confirm_) {
      ROS_INFO(
          "[FrontierExploration] No reachable frontier left: %.1f%% of the area observed, %d "
          "goals, "
          "%zu blocked, %.0f s",
          100.0 * map_.coverage(), goals_sent_, blocked_.size(),
          (ros::Time::now() - start_time_).toSec());
      transit(RETURN_HOME, "explored");
    }
  } else {
    no_frontier_cycles_ = 0;
  }
  publishVisualisation();
}

bool FrontierExploration::selectGoal() {
  const auto& frontiers = map_.frontiers();
  if (frontiers.empty())
    return false;

  const Eigen::Vector2d robot = odom_pos_.head<2>();
  const Eigen::Vector2d vel = odom_vel_.head<2>();
  auto cost = [&](const Eigen::Vector2d& goal, double distance, size_t cells) {
    double c = distance - size_weight_ * std::min(cells * map_.resolution(), 2.0);
    const Eigen::Vector2d dir = goal - robot;
    if (vel.norm() > 0.3 && dir.norm() > 1e-3)
      c += turn_weight_ *
           std::acos(std::max(-1.0, std::min(1.0, vel.normalized().dot(dir.normalized()))));
    return c;
  };

  int best = -1, nearest = -1;
  double best_cost = 0.0;
  for (size_t i = 0; i < frontiers.size(); ++i) {
    if ((frontiers[i].goal - robot).norm() < min_goal_distance_) {
      if (nearest < 0 || frontiers[i].distance < frontiers[nearest].distance)
        nearest = i;
      continue;
    }
    const double c = cost(frontiers[i].goal, frontiers[i].distance, frontiers[i].cells.size());
    if (best < 0 || c < best_cost) {
      best = i;
      best_cost = c;
    }
  }

  if (best < 0) {
    // Only frontiers next to the robot are left, where a forward camera does
    // not look. Move away from them; they are then flown to (and seen) from
    // a distance.
    if (have_goal_)
      return true;
    const Eigen::Vector2d& near = frontiers[nearest].goal;
    const double base = std::atan2(robot(1) - near(1), robot(0) - near(0));
    for (int k = 0; k < 16; ++k) {
      const double a = base + (k % 2 ? 1 : -1) * ((k + 1) / 2) * M_PI / 8;
      const Eigen::Vector2d p = robot + back_off_ * Eigen::Vector2d(std::cos(a), std::sin(a));
      if (map_.distance(p) < 2.0 * back_off_ &&
          (p - near).norm() > (robot - near).norm() + 0.5 * back_off_) {
        ROS_INFO("[FrontierExploration] Only frontiers within %.1f m left, backing off",
                 min_goal_distance_);
        sendGoal(p);
        backing_off_ = true;
        return true;
      }
    }
    // No room to back off (e.g. at the start, when only a small patch is
    // observed): fly to the farthest of them instead of waiting.
    int farthest = -1;
    for (size_t i = 0; i < frontiers.size(); ++i)
      if ((frontiers[i].goal - robot).norm() > reach_radius_ &&
          (farthest < 0 || frontiers[i].distance > frontiers[farthest].distance))
        farthest = i;
    if (farthest >= 0) {
      ROS_INFO(
          "[FrontierExploration] Only frontiers within %.1f m left and no room to back off, flying "
          "to the farthest",
          min_goal_distance_);
      sendGoal(frontiers[farthest].goal);
      return true;
    }
    // All of them within reach_radius (a tiny observed patch, e.g. facing an
    // obstacle at the start): head past the nearest into unknown space,
    // which the planner treats as free; the camera looks where it flies.
    const Eigen::Vector2d dir = near - robot;
    const Eigen::Vector2d past =
        robot +
        min_goal_distance_ * (dir.norm() > 1e-3 ? dir.normalized() : Eigen::Vector2d(1.0, 0.0));
    ROS_INFO("[FrontierExploration] Only frontiers within %.1f m left, heading past them",
             reach_radius_);
    sendGoal(past);
    backing_off_ = true;  // not a frontier goal: kept until reached or stuck
    return true;
  }

  // Each new goal makes the planner start over; changing it too often leaves
  // it replanning (and failing to) instead of flying.
  if ((ros::Time::now() - last_goal_time_).toSec() < min_goal_interval_)
    return true;

  const Eigen::Vector2d& goal = frontiers[best].goal;
  if (have_goal_ && !backing_off_) {
    double d = map_.distance(goal_);
    if (d == FrontierMap::kUnreachable)
      d = (goal_ - robot).norm();
    if ((goal - goal_).norm() < goal_keep_radius_ || best_cost > cost(goal_, d, 0) - switch_margin_)
      return true;  // keep the current goal
  }
  sendGoal(goal);
  return true;
}

void FrontierExploration::sendGoal(const Eigen::Vector2d& goal) {
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();
  msg.pose.position.x = goal(0);
  msg.pose.position.y = goal(1);
  msg.pose.position.z = altitude_;
  msg.pose.orientation.w = 1.0;
  goal_pub_.publish(msg);

  goal_ = goal;
  have_goal_ = true;
  backing_off_ = false;
  last_goal_time_ = ros::Time::now();
  best_distance_ = FrontierMap::kUnreachable;
  progress_time_ = ros::Time::now();
  ++goals_sent_;
  ROS_INFO("[FrontierExploration] Goal %d: [%.2f, %.2f] (path %.1f m)", goals_sent_, goal(0),
           goal(1), map_.distance(goal));
}

bool FrontierExploration::makingProgress(double distance) {
  if (distance < best_distance_ - 0.3) {
    best_distance_ = distance;
    progress_time_ = ros::Time::now();
  }
  return (ros::Time::now() - progress_time_).toSec() < stuck_time_;
}

void FrontierExploration::returnHomeState() {
  const Eigen::Vector2d robot = odom_pos_.head<2>();
  const double reach = std::max(goal_tolerance_, reach_radius_);
  if (route_.empty()) {
    // Back through observed free space in line-of-sight legs, not straight:
    // the planner's map can lose thin obstacles (a ring's tube) that the
    // exploration map keeps.
    map_.update(robot);
    route_ = map_.path(home_, return_step_);
    if (route_.empty())
      route_.push_back(home_);
    route_.back() = home_;
    route_idx_ = 0;
    ROS_INFO("[FrontierExploration] Returning to the start through %zu waypoints", route_.size());
    sendGoal(route_[0]);
    return;
  }
  if (route_idx_ + 1 < route_.size()) {
    const double d = (route_[route_idx_] - robot).norm();
    if (d < reach) {
      sendGoal(route_[++route_idx_]);
    } else if (!makingProgress(d)) {
      ROS_WARN("[FrontierExploration] No progress towards return waypoint %zu, skipping it",
               route_idx_);
      sendGoal(route_[++route_idx_]);
    }
    return;
  }
  // reach_radius, not the waypoint tolerance: the planner stops short of
  // goals this close.
  const double d = (home_ - robot).norm();
  if (d < reach) {
    transit(FINISH, "back at the start");
  } else if (!makingProgress(d)) {
    ROS_WARN("[FrontierExploration] Could not return to the start (%.2f m away)", d);
    transit(FINISH, "return failed");
  }
}

void FrontierExploration::finishState() {
  ROS_INFO(
      "[FrontierExploration] Finished: %.1f%% of the area observed, %d goals, %zu blocked, %.0f s",
      100.0 * map_.coverage(), goals_sent_, blocked_.size(),
      (ros::Time::now() - start_time_).toSec());
  have_goal_ = false;
  triggered_ = false;
  route_.clear();
  transit(WAIT_TRIGGER, "finished");
}

void FrontierExploration::transit(State s, const std::string& why) {
  ROS_INFO("[FrontierExploration] %s -> %s (%s)", state_names_[state_].c_str(),
           state_names_[s].c_str(), why.c_str());
  state_ = s;
}

void FrontierExploration::publishState() {
  std_msgs::String msg;
  msg.data = state_names_[state_];
  state_pub_.publish(msg);
  if (have_slice_) {
    std_msgs::Float32 cov;
    cov.data = map_.coverage();
    coverage_pub_.publish(cov);
  }
}

void FrontierExploration::publishVisualisation() {
  const ros::Time now = ros::Time::now();
  if (map_pub_.getNumSubscribers() > 0) {
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = "world";
    grid.header.stamp = now;
    grid.info.resolution = map_.resolution();
    grid.info.width = map_.width();
    grid.info.height = map_.height();
    grid.info.origin.position.x = map_.origin()(0);
    grid.info.origin.position.y = map_.origin()(1);
    grid.info.origin.position.z = altitude_;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(map_.width() * map_.height());
    for (int y = 0; y < map_.height(); ++y)
      for (int x = 0; x < map_.width(); ++x) {
        const int8_t c = map_.cell(x, y);
        grid.data[x + y * map_.width()] =
            c == FrontierMap::UNKNOWN
                ? -1
                : c == FrontierMap::OCCUPIED
                      ? 100
                      : c == FrontierMap::BLOCKED ? 75 : map_.inflated(x, y) ? 50 : 0;
      }
    map_pub_.publish(grid);
  }

  if (frontier_pub_.getNumSubscribers() > 0) {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker cells, goals, goal;
    cells.header.frame_id = goals.header.frame_id = goal.header.frame_id = "world";
    cells.header.stamp = goals.header.stamp = goal.header.stamp = now;
    cells.ns = "frontier_cells";
    cells.type = visualization_msgs::Marker::CUBE_LIST;
    cells.scale.x = cells.scale.y = map_.resolution();
    cells.scale.z = 0.02;
    goals.ns = "frontier_goals";
    goals.type = visualization_msgs::Marker::SPHERE_LIST;
    goals.scale.x = goals.scale.y = goals.scale.z = 0.25;
    goals.color.r = 1.0;
    goals.color.g = 0.6;
    goals.color.a = 1.0;
    cells.pose.orientation.w = goals.pose.orientation.w = goal.pose.orientation.w = 1.0;
    const auto& frontiers = map_.frontiers();
    for (size_t i = 0; i < frontiers.size(); ++i) {
      std_msgs::ColorRGBA color;
      color.r = 0.2 + 0.8 * std::fmod(i * 0.37, 1.0);
      color.g = 0.9 - 0.6 * std::fmod(i * 0.61, 1.0);
      color.b = 0.3;
      color.a = 0.9;
      for (int a : frontiers[i].cells) {
        geometry_msgs::Point p;
        const Eigen::Vector2d c = map_.cellCentre(a);
        p.x = c(0);
        p.y = c(1);
        p.z = altitude_;
        cells.points.push_back(p);
        cells.colors.push_back(color);
      }
      geometry_msgs::Point g;
      g.x = frontiers[i].goal(0);
      g.y = frontiers[i].goal(1);
      g.z = altitude_;
      goals.points.push_back(g);
    }
    goal.ns = "goal";
    goal.type = visualization_msgs::Marker::SPHERE;
    goal.action = have_goal_ ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
    goal.pose.position.x = goal_(0);
    goal.pose.position.y = goal_(1);
    goal.pose.position.z = altitude_;
    goal.scale.x = goal.scale.y = goal.scale.z = 0.45;
    goal.color.g = 1.0;
    goal.color.a = 0.9;
    markers.markers = {cells, goals, goal};
    frontier_pub_.publish(markers);
  }
}

void FrontierExploration::sliceCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
  map_.merge(Eigen::Vector2d(msg->info.origin.position.x, msg->info.origin.position.y),
             msg->info.resolution, msg->info.width, msg->info.height, msg->data);
  have_slice_ = true;
}

void FrontierExploration::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                              msg->pose.pose.position.z);
  odom_vel_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                              msg->twist.twist.linear.z);
  have_odom_ = true;
}

void FrontierExploration::triggerCallback(const geometry_msgs::PoseStampedConstPtr& /*msg*/) {
  if (state_ == WAIT_TRIGGER)
    triggered_ = true;
}

void FrontierExploration::finishCallback(const std_msgs::EmptyConstPtr& /*msg*/) {
  ROS_INFO("[FrontierExploration] Manual finish command received");
  if (state_ == EXPLORE || state_ == RETURN_HOME)
    transit(FINISH, "finish command");
}

}  // namespace exploration_manager
