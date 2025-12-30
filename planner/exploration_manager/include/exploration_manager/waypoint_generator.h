#ifndef _WAYPOINT_GENERATOR_H_
#define _WAYPOINT_GENERATOR_H_

#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <plan_env/grid_map.h>

using std::shared_ptr;
using std::vector;

namespace exploration_manager {

class WaypointGenerator {
public:
  WaypointGenerator() {}
  ~WaypointGenerator() {}

  void init(ros::NodeHandle& nh, shared_ptr<GridMap>& grid_map);
  
  // Generate waypoints in lawnmower pattern
  void generateWaypoints(const Eigen::Vector3d& start_pos);
  
  // Get generated waypoints
  vector<Eigen::Vector3d> getWaypoints() { return waypoints_; }
  
  // Check if position is valid (free space)
  bool isValidWaypoint(const Eigen::Vector3d& pos);
  
  // Visualization
  void publishWaypoints();
  
private:
  // Map
  shared_ptr<GridMap> grid_map_;
  
  // Parameters
  double resolution_;
  Eigen::Vector3d exploration_center_;
  double exploration_width_;
  double exploration_height_;
  double exploration_altitude_;
  double line_spacing_;  // 0.5m between lines
  
  // Generated waypoints
  vector<Eigen::Vector3d> waypoints_;
  
  // ROS
  ros::Publisher waypoint_pub_;
  ros::NodeHandle nh_;
};

} // namespace exploration_manager

#endif
