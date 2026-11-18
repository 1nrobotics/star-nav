#include "exploration_manager/frontier_map.h"

#include <algorithm>
#include <cmath>
#include <queue>

namespace exploration_manager {

namespace {

const int kDx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
const int kDy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

}  // namespace

constexpr double FrontierMap::kUnreachable;

void FrontierMap::init(const Eigen::Vector2d& area_min, const Eigen::Vector2d& area_max,
                       const Params& params) {
  p_ = params;
  // Margin: obstacles just outside the area still constrain paths inside it.
  const double margin = p_.inflation + 1.0;
  origin_ = area_min - Eigen::Vector2d::Constant(margin);
  const Eigen::Vector2d extent = area_max - area_min + Eigen::Vector2d::Constant(2.0 * margin);
  size_ = (extent / p_.resolution).array().ceil().cast<int>();
  // Area cells: those whose centre is in the area.
  area_lo_ = ((area_min - origin_) / p_.resolution - Eigen::Vector2d::Constant(0.5 - 1e-9))
                 .array()
                 .ceil()
                 .cast<int>();
  area_hi_ = ((area_max - origin_) / p_.resolution - Eigen::Vector2d::Constant(0.5 + 1e-9))
                 .array()
                 .floor()
                 .cast<int>();
  const int n = size_(0) * size_(1);
  cells_.assign(n, UNKNOWN);
  inflated_.assign(n, 0);
  dist_.assign(n, kUnreachable);
  is_frontier_.assign(n, 0);
  frontiers_.clear();
  robot_free_ = false;
}

bool FrontierMap::toCell(const Eigen::Vector2d& p, Eigen::Vector2i& c) const {
  c = ((p - origin_) / p_.resolution).array().floor().cast<int>();
  return inGrid(c);
}

bool FrontierMap::inGrid(const Eigen::Vector2i& c) const {
  return c(0) >= 0 && c(1) >= 0 && c(0) < size_(0) && c(1) < size_(1);
}

bool FrontierMap::inArea(const Eigen::Vector2i& c) const {
  return c(0) >= area_lo_(0) && c(1) >= area_lo_(1) && c(0) <= area_hi_(0) && c(1) <= area_hi_(1);
}

Eigen::Vector2d FrontierMap::cellCentre(int adr) const {
  return origin_ +
         (Eigen::Vector2d(adr % size_(0), adr / size_(0)) + Eigen::Vector2d::Constant(0.5)) *
             p_.resolution;
}

void FrontierMap::merge(const Eigen::Vector2d& origin, double resolution, int width, int height,
                        const std::vector<int8_t>& data) {
  if ((int)data.size() < width * height)
    return;
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x) {
      const int8_t v = data[x + y * width];
      if (v < 0)
        continue;
      Eigen::Vector2i c;
      if (!toCell(origin + (Eigen::Vector2d(x, y) + Eigen::Vector2d::Constant(0.5)) * resolution,
                  c))
        continue;
      cells_[address(c)] = v > 50 ? OCCUPIED : FREE;
    }
}

void FrontierMap::inflate() {
  std::fill(inflated_.begin(), inflated_.end(), 0);
  const int r = (int)std::ceil(p_.inflation / p_.resolution - 1e-6);
  std::vector<Eigen::Vector2i> disk;
  for (int dy = -r; dy <= r; ++dy)
    for (int dx = -r; dx <= r; ++dx)
      if (dx * dx + dy * dy <= r * r)
        disk.emplace_back(dx, dy);
  for (int y = 0; y < size_(1); ++y)
    for (int x = 0; x < size_(0); ++x) {
      if (cells_[x + y * size_(0)] != OCCUPIED)
        continue;
      for (const auto& d : disk) {
        const Eigen::Vector2i c(x + d(0), y + d(1));
        if (inGrid(c))
          inflated_[address(c)] = 1;
      }
    }
}

// Dijkstra over traversable cells. A robot inside an inflated (or not yet
// observed) cell first searches up to escape_radius through cells that are
// not occupied, and starts from the traversable cells found there.
void FrontierMap::reach(const Eigen::Vector2d& robot) {
  std::fill(dist_.begin(), dist_.end(), kUnreachable);
  Eigen::Vector2i start;
  toCell(robot, start);
  start = start.cwiseMax(Eigen::Vector2i::Zero()).cwiseMin(size_ - Eigen::Vector2i::Ones());

  typedef std::pair<double, int> Item;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  const int s = address(start);
  if (traversable(s)) {
    dist_[s] = 0.0;
    open.emplace(0.0, s);
  } else {
    std::vector<double> escape(cells_.size(), kUnreachable);
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
    escape[s] = 0.0;
    q.emplace(0.0, s);
    while (!q.empty()) {
      const Item it = q.top();
      q.pop();
      if (it.first > escape[it.second])
        continue;
      if (traversable(it.second)) {
        dist_[it.second] = it.first;
        open.emplace(it.first, it.second);
        continue;  // escape ends at free space
      }
      const Eigen::Vector2i c(it.second % size_(0), it.second / size_(0));
      for (int k = 0; k < 8; ++k) {
        const Eigen::Vector2i n(c(0) + kDx8[k], c(1) + kDy8[k]);
        if (!inGrid(n) || cells_[address(n)] == OCCUPIED)
          continue;
        const double d = it.first + (k < 4 ? 1.0 : M_SQRT2) * p_.resolution;
        if (d <= p_.escape_radius && d < escape[address(n)]) {
          escape[address(n)] = d;
          q.emplace(d, address(n));
        }
      }
    }
  }
  robot_free_ = !open.empty();

  while (!open.empty()) {
    const Item it = open.top();
    open.pop();
    if (it.first > dist_[it.second])
      continue;
    const Eigen::Vector2i c(it.second % size_(0), it.second / size_(0));
    for (int k = 0; k < 8; ++k) {
      const Eigen::Vector2i n(c(0) + kDx8[k], c(1) + kDy8[k]);
      if (!inGrid(n))
        continue;
      const int a = address(n);
      if (!traversable(a))
        continue;
      // No corner cutting between two blocked cells.
      if (k >= 4 && (!traversable(address(Eigen::Vector2i(c(0) + kDx8[k], c(1)))) ||
                     !traversable(address(Eigen::Vector2i(c(0), c(1) + kDy8[k])))))
        continue;
      const double d = it.first + (k < 4 ? 1.0 : M_SQRT2) * p_.resolution;
      if (d < dist_[a]) {
        dist_[a] = d;
        open.emplace(d, a);
      }
    }
  }
}

void FrontierMap::findFrontiers(const std::vector<Eigen::Vector2d>& blocked,
                                double blocked_radius) {
  std::fill(is_frontier_.begin(), is_frontier_.end(), 0);
  frontiers_.clear();

  // An unknown cell counts if it is in the area and is not an isolated hole
  // (all four neighbours observed free): sensor noise leaves such holes.
  auto unknown = [&](const Eigen::Vector2i& c) {
    if (!inArea(c) || cells_[address(c)] != UNKNOWN)
      return false;
    for (int k = 0; k < 4; ++k) {
      const Eigen::Vector2i n(c(0) + kDx8[k], c(1) + kDy8[k]);
      if (!inGrid(n) || cells_[address(n)] != FREE)
        return true;
    }
    return false;
  };

  std::vector<int> candidates;
  for (int y = area_lo_(1); y <= area_hi_(1); ++y)
    for (int x = area_lo_(0); x <= area_hi_(0); ++x) {
      const int a = x + y * size_(0);
      if (dist_[a] == kUnreachable)
        continue;
      bool next_to_unknown = false;
      for (int k = 0; k < 4 && !next_to_unknown; ++k)
        next_to_unknown = unknown(Eigen::Vector2i(x + kDx8[k], y + kDy8[k]));
      if (!next_to_unknown)
        continue;
      const Eigen::Vector2d p = cellCentre(a);
      bool skip = false;
      for (const auto& b : blocked)
        skip = skip || (p - b).norm() < blocked_radius;
      if (skip)
        continue;
      is_frontier_[a] = 1;
      candidates.push_back(a);
    }

  // 8-connected clusters.
  std::vector<uint8_t> seen(cells_.size(), 0);
  for (int seed : candidates) {
    if (seen[seed])
      continue;
    std::vector<int> cluster;
    std::vector<int> stack{seed};
    seen[seed] = 1;
    while (!stack.empty()) {
      const int a = stack.back();
      stack.pop_back();
      cluster.push_back(a);
      const Eigen::Vector2i c(a % size_(0), a / size_(0));
      for (int k = 0; k < 8; ++k) {
        const Eigen::Vector2i n(c(0) + kDx8[k], c(1) + kDy8[k]);
        if (!inGrid(n))
          continue;
        const int b = address(n);
        if (is_frontier_[b] && !seen[b]) {
          seen[b] = 1;
          stack.push_back(b);
        }
      }
    }
    if ((int)cluster.size() >= p_.min_frontier_cells)
      addCluster(cluster);
  }
}

// Splits clusters wider than max_cluster_size along their longer side, then
// takes the cell nearest each part's centroid as its goal.
void FrontierMap::addCluster(std::vector<int>& cells) {
  Eigen::Vector2d lo = Eigen::Vector2d::Constant(1e18), hi = -lo, mean = Eigen::Vector2d::Zero();
  for (int a : cells) {
    const Eigen::Vector2d p = cellCentre(a);
    lo = lo.cwiseMin(p);
    hi = hi.cwiseMax(p);
    mean += p;
  }
  mean /= cells.size();
  const Eigen::Vector2d extent = hi - lo;
  const int axis = extent(0) >= extent(1) ? 0 : 1;
  if (extent(axis) > p_.max_cluster_size && (int)cells.size() >= 2 * p_.min_frontier_cells) {
    std::vector<int> a, b;
    for (int c : cells)
      (cellCentre(c)(axis) < mean(axis) ? a : b).push_back(c);
    if (!a.empty() && !b.empty()) {
      if ((int)a.size() >= p_.min_frontier_cells)
        addCluster(a);
      if ((int)b.size() >= p_.min_frontier_cells)
        addCluster(b);
      return;
    }
  }
  Frontier f;
  f.cells = cells;
  double best = 1e18;
  for (int a : cells) {
    const double d = (cellCentre(a) - mean).squaredNorm();
    if (d < best) {
      best = d;
      f.goal = cellCentre(a);
      f.distance = dist_[a];
    }
  }
  frontiers_.push_back(f);
}

void FrontierMap::update(const Eigen::Vector2d& robot, const std::vector<Eigen::Vector2d>& blocked,
                         double blocked_radius) {
  inflate();
  reach(robot);
  findFrontiers(blocked, blocked_radius);
}

double FrontierMap::distance(const Eigen::Vector2d& p) const {
  Eigen::Vector2i c;
  if (!toCell(p, c))
    return kUnreachable;
  return dist_[address(c)];
}

bool FrontierMap::nearFrontier(const Eigen::Vector2d& p, double radius) const {
  const int r = (int)std::ceil(radius / p_.resolution);
  Eigen::Vector2i c;
  toCell(p, c);
  for (int dy = -r; dy <= r; ++dy)
    for (int dx = -r; dx <= r; ++dx) {
      const Eigen::Vector2i n(c(0) + dx, c(1) + dy);
      if (inGrid(n) && is_frontier_[address(n)] && (cellCentre(address(n)) - p).norm() <= radius)
        return true;
    }
  return false;
}

double FrontierMap::coverage() const {
  int known = 0, total = 0;
  for (int y = area_lo_(1); y <= area_hi_(1); ++y)
    for (int x = area_lo_(0); x <= area_hi_(0); ++x) {
      ++total;
      known += cells_[x + y * size_(0)] != UNKNOWN;
    }
  return total ? double(known) / total : 0.0;
}

}  // namespace exploration_manager
