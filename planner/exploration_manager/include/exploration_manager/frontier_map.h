#ifndef _FRONTIER_MAP_H_
#define _FRONTIER_MAP_H_

#include <Eigen/Eigen>
#include <cstdint>
#include <limits>
#include <vector>

namespace exploration_manager {

// 2D exploration memory at flight height, independent of ROS.
//
// It covers the exploration area plus a margin and keeps every observation
// (the planner's map may forget what lies outside its sliding window).
// update() then finds, from the robot's position:
//   * which cells are reachable through observed free space, keeping
//     `inflation` away from obstacles (Dijkstra, 8-connected), and
//   * the frontiers: reachable free cells inside the area next to unknown
//     cells inside the area, grouped into clusters with a goal each.
class FrontierMap {
 public:
  // BLOCKED: observed, not an obstacle itself but within the planner's
  // inflation (the slice's values 1-99). Not traversable, but a robot inside
  // it can leave through it.
  enum : int8_t { UNKNOWN = -1, FREE = 0, OCCUPIED = 1, BLOCKED = 2 };

  struct Params {
    double resolution = 0.1;
    double inflation = 0.3;         // m kept from obstacles
    int min_frontier_cells = 8;     // smaller clusters are ignored
    double max_cluster_size = 3.0;  // m: larger clusters are split
    double escape_radius = 1.0;  // m searched for free space when the robot is in an inflated cell
  };

  struct Frontier {
    std::vector<int> cells;  // cell addresses
    Eigen::Vector2d goal;    // the cluster cell nearest its centroid
    double distance;         // path length from the robot to the goal
  };

  static constexpr double kUnreachable = std::numeric_limits<double>::infinity();

  // The exploration area is [area_min, area_max].
  void init(const Eigen::Vector2d& area_min, const Eigen::Vector2d& area_max, const Params& params);

  // Merges a grid (-1 unknown, 0 free, 1-99 blocked, 100 occupied; row-major,
  // x fastest, origin at the corner of cell 0). Unknown cells keep what was
  // there.
  void merge(const Eigen::Vector2d& origin, double resolution, int width, int height,
             const std::vector<int8_t>& data);

  // Recomputes reachability and frontiers from the robot's position.
  // Frontier cells within `radius` of a blocked point are dropped.
  void update(const Eigen::Vector2d& robot, const std::vector<Eigen::Vector2d>& blocked = {},
              double blocked_radius = 0.0);

  const std::vector<Frontier>& frontiers() const {
    return frontiers_;
  }
  // Path length from the robot (last update) to p; kUnreachable if none.
  double distance(const Eigen::Vector2d& p) const;
  // True if a frontier cell lies within `radius` of p (last update).
  bool nearFrontier(const Eigen::Vector2d& p, double radius) const;
  // False if the robot was neither in nor near reachable free space.
  bool robotInFreeSpace() const {
    return robot_free_;
  }
  // Fraction of the exploration area observed (free or occupied).
  double coverage() const;

  // Grid access (for visualisation and tests).
  int width() const {
    return size_(0);
  }
  int height() const {
    return size_(1);
  }
  const Eigen::Vector2d& origin() const {
    return origin_;
  }
  double resolution() const {
    return p_.resolution;
  }
  int8_t cell(int x, int y) const {
    return cells_[x + y * size_(0)];
  }
  bool inflated(int x, int y) const {
    return inflated_[x + y * size_(0)] != 0;
  }
  Eigen::Vector2d cellCentre(int address) const;

 private:
  bool toCell(const Eigen::Vector2d& p, Eigen::Vector2i& c) const;
  int address(const Eigen::Vector2i& c) const {
    return c(0) + c(1) * size_(0);
  }
  bool inGrid(const Eigen::Vector2i& c) const;
  bool inArea(const Eigen::Vector2i& c) const;
  bool traversable(int adr) const {
    return cells_[adr] == FREE && !inflated_[adr];
  }
  void inflate();
  void reach(const Eigen::Vector2d& robot);
  void findFrontiers(const std::vector<Eigen::Vector2d>& blocked, double blocked_radius);
  void addCluster(std::vector<int>& cells);

  Params p_;
  Eigen::Vector2d origin_;             // corner of cell (0, 0)
  Eigen::Vector2i size_;               // cells
  Eigen::Vector2i area_lo_, area_hi_;  // exploration area, cells (inclusive)
  std::vector<int8_t> cells_;
  std::vector<uint8_t> inflated_;
  std::vector<double> dist_;
  std::vector<Frontier> frontiers_;
  std::vector<uint8_t> is_frontier_;
  bool robot_free_ = false;
};

}  // namespace exploration_manager

#endif
