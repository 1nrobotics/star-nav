#include "exploration_manager/waypoint_generator.h"

namespace exploration_manager {

void WaypointGenerator::init(ros::NodeHandle& nh, shared_ptr<GridMap>& grid_map) {
  nh_ = nh;
  grid_map_ = grid_map;

  // Load parameters
  nh.param("exploration/center_x", exploration_center_(0), 0.0);
  nh.param("exploration/center_y", exploration_center_(1), 0.0);
  nh.param("exploration/center_z", exploration_center_(2), 1.5);
  nh.param("exploration/width", exploration_width_, 10.0);
  nh.param("exploration/height", exploration_height_, 10.0);
  nh.param("exploration/altitude", exploration_altitude_, 1.0);
  nh.param("exploration/line_spacing", line_spacing_, 0.5);

  // Get map resolution
  resolution_ = grid_map_->getResolution();

  // Publisher for waypoint visualization
  waypoint_pub_ = nh.advertise<visualization_msgs::MarkerArray>("exploration/waypoints", 10, true);

  ROS_INFO("[WaypointGenerator] Initialized");
  ROS_INFO("  - Exploration area: %.1fx%.1f m at [%.1f, %.1f, %.1f]",
           exploration_width_, exploration_height_,
           exploration_center_(0), exploration_center_(1), exploration_center_(2));
  ROS_INFO("  - Line spacing: %.2f m", line_spacing_);
}

void WaypointGenerator::generateWaypoints(const Eigen::Vector3d& start_pos) {
  waypoints_.clear();

  // Calculate rectangle bounds
  double half_width = exploration_width_ / 2.0;
  double half_height = exploration_height_ / 2.0;

  double min_x = exploration_center_(0) - half_width;
  double max_x = exploration_center_(0) + half_width;
  double min_y = exploration_center_(1) - half_height;
  double max_y = exploration_center_(1) + half_height;

  // Generate lawnmower pattern
  // Start from bottom-left, move right, then up, then left, etc.
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

  ROS_INFO("[WaypointGenerator] Generated %lu waypoints in lawnmower pattern", waypoints_.size());

  // Publish waypoints for visualization
  publishWaypoints();
}

bool WaypointGenerator::isValidWaypoint(const Eigen::Vector3d& pos) {
  // Not inside an inflated obstacle. Outside the stored map (-1, e.g. beyond
  // a sliding map's window) nothing is known yet, so the waypoint is kept.
  if (grid_map_->getInflateOccupancy(pos) == 1) {
    return false;
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

void WaypointGenerator::publishWaypoints() {
  visualization_msgs::MarkerArray marker_array;

  for (size_t i = 0; i < waypoints_.size(); ++i) {
    // Sphere marker for waypoint
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.ns = "waypoints";
    sphere.id = i;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;

    sphere.pose.position.x = waypoints_[i](0);
    sphere.pose.position.y = waypoints_[i](1);
    sphere.pose.position.z = waypoints_[i](2);
    sphere.pose.orientation.w = 1.0;

    sphere.scale.x = 0.3;
    sphere.scale.y = 0.3;
    sphere.scale.z = 0.3;

    // Color based on sequence (gradient from blue to red)
    float ratio = (float)i / waypoints_.size();
    sphere.color.r = ratio;
    sphere.color.g = 0.5;
    sphere.color.b = 1.0 - ratio;
    sphere.color.a = 0.8;

    marker_array.markers.push_back(sphere);

    // Text label showing waypoint number
    visualization_msgs::Marker text;
    text.header.frame_id = "world";
    text.header.stamp = ros::Time::now();
    text.ns = "waypoints";
    text.id = i + 1000;  // Offset to avoid ID conflicts
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;

    text.pose.position.x = waypoints_[i](0);
    text.pose.position.y = waypoints_[i](1);
    text.pose.position.z = waypoints_[i](2) + 0.5;
    text.pose.orientation.w = 1.0;

    text.scale.z = 0.3;

    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;

    std::stringstream ss;
    ss << i;
    text.text = ss.str();

    marker_array.markers.push_back(text);
  }

  waypoint_pub_.publish(marker_array);
}

} // namespace exploration_manager
