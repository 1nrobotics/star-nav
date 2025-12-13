#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <list>
#include <memory>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <plan_env/grid_map.h>

using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using std::list;

namespace exploration_manager {

// Viewpoint for exploration
struct Viewpoint {
  Eigen::Vector3d pos_;
  double yaw_;
  
  Viewpoint() {}
  Viewpoint(Eigen::Vector3d pos, double yaw) : pos_(pos), yaw_(yaw) {}
};

// Frontier cluster
class Frontier {
public:
  vector<Eigen::Vector3d> cells_;       // Frontier cells (voxel centers)
  Eigen::Vector3d average_;             // Average position of frontier
  vector<Viewpoint> viewpoints_;        // Candidate viewpoints to observe this frontier
  double cost_;                         // Cost to reach this frontier
  int id_;                             // Frontier ID
  
  Frontier() {
    cost_ = 0.0;
    id_ = -1;
  }
  
  void calcAverage() {
    if (cells_.empty()) return;
    average_ = Eigen::Vector3d::Zero();
    for (auto& cell : cells_) {
      average_ += cell;
    }
    average_ /= cells_.size();
  }
};

class FrontierFinder {
public:
  FrontierFinder() {}
  ~FrontierFinder() {}

  void init(ros::NodeHandle& nh, shared_ptr<GridMap>& grid_map);
  
  // Main frontier search function
  void searchFrontiers();
  
  // Get frontiers sorted by cost/gain
  vector<Frontier> getFrontiers() { return frontiers_; }
  
  // Get best frontier for exploration
  Frontier getBestFrontier(const Eigen::Vector3d& cur_pos, const Eigen::Vector3d& cur_vel);
  
  // Check if exploration is complete
  bool isExplorationComplete() { return frontiers_.empty(); }
  
  // Visualization
  void publishFrontiers();
  void publishFrontierCells(const vector<Eigen::Vector3d>& cells);
  
private:
  // Map
  shared_ptr<GridMap> grid_map_;
  
  // Parameters
  double resolution_;
  Eigen::Vector3d map_min_, map_max_;
  double frontier_cluster_min_size_;
  double frontier_search_radius_;
  int frontier_min_cells_;
  double viewpoint_distance_;
  int viewpoint_num_;
  
  // Frontiers
  vector<Frontier> frontiers_;
  vector<Eigen::Vector3d> frontier_cells_;
  int frontier_id_counter_;
  
  // Helper functions
  bool isNeighborUnknown(const Eigen::Vector3d& pos);
  bool isFrontierCell(const Eigen::Vector3d& pos);
  void clusterFrontiers();
  void computeViewpoints(Frontier& frontier);
  double computeFrontierCost(const Frontier& frontier, const Eigen::Vector3d& cur_pos, 
                             const Eigen::Vector3d& cur_vel);
  
  // Visited flags for clustering
  vector<vector<vector<int>>> visited_flag_;
  
  // ROS
  ros::Publisher frontier_pub_, frontier_cells_pub_;
  ros::NodeHandle nh_;
};

} // namespace exploration_manager

#endif
