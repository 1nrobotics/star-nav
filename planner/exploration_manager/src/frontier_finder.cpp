#include "exploration_manager/frontier_finder.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

namespace exploration_manager {

void FrontierFinder::init(ros::NodeHandle& nh, shared_ptr<GridMap>& grid_map) {
  nh_ = nh;
  grid_map_ = grid_map;
  frontier_id_counter_ = 0;
  
  // Load parameters
  nh.param("exploration/frontier_cluster_min_size", frontier_cluster_min_size_, 0.5);
  nh.param("exploration/frontier_search_radius", frontier_search_radius_, 5.0);
  nh.param("exploration/frontier_min_cells", frontier_min_cells_, 5);
  nh.param("exploration/viewpoint_distance", viewpoint_distance_, 2.0);
  nh.param("exploration/viewpoint_num", viewpoint_num_, 8);
  
  // Geofence parameters
  nh.param("exploration/use_geofence", use_geofence_, true);
  nh.param("exploration/max_exploration_radius", max_exploration_radius_, 25.0);
  nh.param("exploration/exploration_center_x", exploration_center_(0), 0.0);
  nh.param("exploration/exploration_center_y", exploration_center_(1), 0.0);
  nh.param("exploration/exploration_center_z", exploration_center_(2), 1.5);
  
  // Explored region tracking
  nh.param("exploration/explored_cell_expiry_time", explored_cell_expiry_time_, 0.0); // 0 = never revisit
  
  // Get map parameters
  grid_map_->getRegion(map_min_, map_max_);
  resolution_ = grid_map_->getResolution();
  
  // Initialize visited flags
  Eigen::Vector3i map_size = Eigen::Vector3i(
    ceil((map_max_(0) - map_min_(0)) / resolution_),
    ceil((map_max_(1) - map_min_(1)) / resolution_),
    ceil((map_max_(2) - map_min_(2)) / resolution_)
  );
  
  visited_flag_.resize(map_size(0));
  for (int i = 0; i < map_size(0); ++i) {
    visited_flag_[i].resize(map_size(1));
    for (int j = 0; j < map_size(1); ++j) {
      visited_flag_[i][j].resize(map_size(2), 0);
    }
  }
  
  // Publishers
  frontier_pub_ = nh.advertise<visualization_msgs::MarkerArray>("exploration/frontiers", 10);
  frontier_cells_pub_ = nh.advertise<sensor_msgs::PointCloud2>("exploration/frontier_cells", 10);
  
  ROS_INFO("[FrontierFinder] Initialized:");
  ROS_INFO("  - Search radius: %.2f m", frontier_search_radius_);
  ROS_INFO("  - Geofence: %s", use_geofence_ ? "ENABLED" : "DISABLED");
  if (use_geofence_) {
    ROS_INFO("  - Max exploration radius: %.2f m from [%.1f, %.1f, %.1f]", 
             max_exploration_radius_, exploration_center_(0), exploration_center_(1), exploration_center_(2));
  }
  ROS_INFO("  - Explored region tracking: %s", 
           explored_cell_expiry_time_ > 0 ? "ENABLED (with expiry)" : "ENABLED (permanent)");
}

bool FrontierFinder::isFrontierCell(const Eigen::Vector3d& pos) {
  // A frontier cell is:
  // 1. Free space
  // 2. Has at least one unknown neighbor
  // 3. Within geofence (if enabled)
  // 4. Not already explored (if tracking enabled)
  
  // Check geofence first (fast check)
  if (!isWithinGeofence(pos)) {
    return false;
  }
  
  // Check if already explored
  if (isAlreadyExplored(pos)) {
    return false;
  }
  
  if (grid_map_->getInflateOccupancy(pos) == 1) {
    return false; // Occupied
  }
  
  if (grid_map_->getOccupancy(pos) == -1) {
    return false; // Unknown itself
  }
  
  return isNeighborUnknown(pos);
}

bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3d& pos) {
  // Check 6-connected neighbors
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (abs(dx) + abs(dy) + abs(dz) != 1) continue; // Only 6-connected
        
        Eigen::Vector3d neighbor = pos + Eigen::Vector3d(dx, dy, dz) * resolution_;
        
        if (grid_map_->getOccupancy(neighbor) == -1) {
          return true; // Has unknown neighbor
        }
      }
    }
  }
  return false;
}

void FrontierFinder::searchFrontiers() {
  frontier_cells_.clear();
  frontiers_.clear();
  
  ros::Time t1 = ros::Time::now();
  
  // Reset visited flags
  for (auto& plane : visited_flag_) {
    for (auto& row : plane) {
      std::fill(row.begin(), row.end(), 0);
    }
  }
  
  // Get current map bounds (observed region)
  Eigen::Vector3d search_min, search_max;
  grid_map_->getRegion(search_min, search_max);
  
  // Search for frontier cells
  for (double x = search_min(0); x < search_max(0); x += resolution_) {
    for (double y = search_min(1); y < search_max(1); y += resolution_) {
      for (double z = search_min(2); z < search_max(2); z += resolution_) {
        Eigen::Vector3d pos(x, y, z);
        
        if (isFrontierCell(pos)) {
          frontier_cells_.push_back(pos);
        }
      }
    }
  }
  
  ROS_INFO("[FrontierFinder] Found %lu frontier cells", frontier_cells_.size());
  
  // Cluster frontier cells
  if (!frontier_cells_.empty()) {
    clusterFrontiers();
  }
  
  ros::Time t2 = ros::Time::now();
  ROS_INFO("[FrontierFinder] Search took %.3f ms, found %lu frontiers", 
           (t2 - t1).toSec() * 1000.0, frontiers_.size());
}

void FrontierFinder::clusterFrontiers() {
  // Simple clustering using BFS
  list<Eigen::Vector3d> cell_queue;
  vector<vector<Eigen::Vector3d>> clusters;
  
  for (const auto& cell : frontier_cells_) {
    Eigen::Vector3i idx;
    grid_map_->posToIndex(cell, idx);
    
    if (visited_flag_[idx(0)][idx(1)][idx(2)] == 1) continue;
    
    // Start new cluster
    vector<Eigen::Vector3d> cluster;
    cell_queue.push_back(cell);
    visited_flag_[idx(0)][idx(1)][idx(2)] = 1;
    
    while (!cell_queue.empty()) {
      Eigen::Vector3d cur = cell_queue.front();
      cell_queue.pop_front();
      cluster.push_back(cur);
      
      // Check neighbors
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (abs(dx) + abs(dy) + abs(dz) != 1) continue;
            
            Eigen::Vector3d neighbor = cur + Eigen::Vector3d(dx, dy, dz) * resolution_;
            Eigen::Vector3i n_idx;
            grid_map_->posToIndex(neighbor, n_idx);
            
            if (n_idx(0) < 0 || n_idx(0) >= (int)visited_flag_.size() ||
                n_idx(1) < 0 || n_idx(1) >= (int)visited_flag_[0].size() ||
                n_idx(2) < 0 || n_idx(2) >= (int)visited_flag_[0][0].size()) {
              continue;
            }
            
            if (visited_flag_[n_idx(0)][n_idx(1)][n_idx(2)] == 1) continue;
            
            if (isFrontierCell(neighbor)) {
              cell_queue.push_back(neighbor);
              visited_flag_[n_idx(0)][n_idx(1)][n_idx(2)] = 1;
            }
          }
        }
      }
    }
    
    // Add cluster if large enough
    if ((int)cluster.size() >= frontier_min_cells_) {
      clusters.push_back(cluster);
    }
  }
  
  // Create Frontier objects
  for (const auto& cluster : clusters) {
    Frontier frontier;
    frontier.cells_ = cluster;
    frontier.calcAverage();
    frontier.id_ = frontier_id_counter_++;
    
    // Compute viewpoints
    computeViewpoints(frontier);
    
    if (!frontier.viewpoints_.empty()) {
      frontiers_.push_back(frontier);
    }
  }
  
  ROS_INFO("[FrontierFinder] Clustered into %lu frontiers", frontiers_.size());
}

void FrontierFinder::computeViewpoints(Frontier& frontier) {
  // Generate viewpoints around the frontier
  frontier.viewpoints_.clear();
  
  for (int i = 0; i < viewpoint_num_; ++i) {
    double yaw = 2.0 * M_PI * i / viewpoint_num_;
    Eigen::Vector3d dir(cos(yaw), sin(yaw), 0);
    Eigen::Vector3d vp = frontier.average_ + dir * viewpoint_distance_;
    
    // Check if viewpoint is in free space
    if (grid_map_->getInflateOccupancy(vp) == 0) {
      frontier.viewpoints_.emplace_back(vp, yaw);
    }
  }
}

double FrontierFinder::computeFrontierCost(const Frontier& frontier, 
                                           const Eigen::Vector3d& cur_pos,
                                           const Eigen::Vector3d& cur_vel) {
  // Cost = distance - lambda * size
  // Lower cost = better frontier
  double dist = (frontier.average_ - cur_pos).norm();
  double size_gain = frontier.cells_.size() * 0.1; // Reward larger frontiers
  
  return dist - size_gain;
}

Frontier FrontierFinder::getBestFrontier(const Eigen::Vector3d& cur_pos, 
                                         const Eigen::Vector3d& cur_vel) {
  if (frontiers_.empty()) {
    ROS_WARN("[FrontierFinder] No frontiers available!");
    return Frontier();
  }
  
  // Compute costs for all frontiers
  double min_cost = std::numeric_limits<double>::max();
  int best_idx = 0;
  
  for (size_t i = 0; i < frontiers_.size(); ++i) {
    frontiers_[i].cost_ = computeFrontierCost(frontiers_[i], cur_pos, cur_vel);
    
    if (frontiers_[i].cost_ < min_cost) {
      min_cost = frontiers_[i].cost_;
      best_idx = i;
    }
  }
  
  ROS_INFO("[FrontierFinder] Best frontier: ID=%d, cost=%.2f, cells=%lu", 
           frontiers_[best_idx].id_, min_cost, frontiers_[best_idx].cells_.size());
  
  return frontiers_[best_idx];
}

void FrontierFinder::publishFrontiers() {
  visualization_msgs::MarkerArray marker_array;
  
  for (size_t i = 0; i < frontiers_.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "frontiers";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = frontiers_[i].average_(0);
    marker.pose.position.y = frontiers_[i].average_(1);
    marker.pose.position.z = frontiers_[i].average_(2);
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    marker_array.markers.push_back(marker);
  }
  
  frontier_pub_.publish(marker_array);
}

void FrontierFinder::publishFrontierCells(const vector<Eigen::Vector3d>& cells) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (const auto& cell : cells) {
    pcl::PointXYZ pt;
    pt.x = cell(0);
    pt.y = cell(1);
    pt.z = cell(2);
    cloud.points.push_back(pt);
  }
  
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "odom";
  msg.header.stamp = ros::Time::now();
  
  frontier_cells_pub_.publish(msg);
}

bool FrontierFinder::isWithinGeofence(const Eigen::Vector3d& pos) {
  if (!use_geofence_) {
    return true; // No geofence, everything is allowed
  }
  
  // Check if position is within max_exploration_radius_ from exploration_center_
  double dist = (pos - exploration_center_).norm();
  return dist <= max_exploration_radius_;
}

bool FrontierFinder::isAlreadyExplored(const Eigen::Vector3d& pos) {
  // Convert position to grid index
  Eigen::Vector3i idx;
  grid_map_->posToIndex(pos, idx);
  
  // Check if this cell was already explored
  auto it = explored_cells_.find(idx);
  if (it == explored_cells_.end()) {
    return false; // Not explored yet
  }
  
  // Check if exploration expired (if expiry time is set)
  if (explored_cell_expiry_time_ > 0.0) {
    ros::Time now = ros::Time::now();
    ros::Duration elapsed = now - it->second;
    if (elapsed.toSec() > explored_cell_expiry_time_) {
      explored_cells_.erase(it); // Remove expired entry
      return false; // Can explore again
    }
  }
  
  return true; // Already explored and not expired
}

void FrontierFinder::markAsExplored(const Eigen::Vector3d& pos) {
  // Convert position to grid index
  Eigen::Vector3i idx;
  grid_map_->posToIndex(pos, idx);
  
  // Mark with current timestamp
  explored_cells_[idx] = ros::Time::now();
  
  // Also mark nearby cells (to account for viewpoint observation range)
  double mark_radius = viewpoint_distance_ * 1.5; // Mark cells within 1.5x viewpoint distance
  for (double dx = -mark_radius; dx <= mark_radius; dx += resolution_) {
    for (double dy = -mark_radius; dy <= mark_radius; dy += resolution_) {
      for (double dz = -mark_radius; dz <= mark_radius; dz += resolution_) {
        Eigen::Vector3d nearby = pos + Eigen::Vector3d(dx, dy, dz);
        double dist = (nearby - pos).norm();
        if (dist <= mark_radius) {
          Eigen::Vector3i nearby_idx;
          grid_map_->posToIndex(nearby, nearby_idx);
          explored_cells_[nearby_idx] = ros::Time::now();
        }
      }
    }
  }
}

} // namespace exploration_manager
