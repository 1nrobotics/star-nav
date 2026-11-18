// Tests for GridMap's map update: incremental inflation, hit counting per
// scan, point-cloud ray casting and the sliding map. Needs a ROS master
// (test_grid_map.test); the callbacks are called directly, without spinning.
#include <gtest/gtest.h>

#include <random>

#include <boost/make_shared.hpp>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>

class GridMapTestAccess {
public:
  explicit GridMapTestAccess(GridMap& map) : m_(map) {}

  void odom(const Eigen::Vector3d& p) {
    nav_msgs::Odometry o;
    o.pose.pose.position.x = p.x();
    o.pose.pose.position.y = p.y();
    o.pose.pose.position.z = p.z();
    o.pose.pose.orientation.w = 1.0;
    m_.odomCallback(boost::make_shared<nav_msgs::Odometry>(o));
  }

  void cloud(const std::vector<Eigen::Vector3d>& pts) {
    pcl::PointCloud<pcl::PointXYZ> c;
    for (const auto& p : pts) c.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    auto msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(c, *msg);
    m_.cloudCallback(msg);
  }

  void update() { m_.updateOccupancyCallback(ros::TimerEvent()); }

  // One scan: cloud then the timer's update.
  void scan(const std::vector<Eigen::Vector3d>& pts) {
    cloud(pts);
    update();
  }

  void setOccupied(const Eigen::Vector3i& id, bool occupied) {
    m_.setLogOdds(m_.toAddress(id), id, occupied ? m_.mp_.clamp_max_log_ : m_.mp_.clamp_min_log_);
  }

  void slide(const Eigen::Vector3d& center) { m_.slideWindow(center); }

  Eigen::Vector3i windowMin() const { return m_.md_.window_min_; }
  Eigen::Vector3i windowSize() const { return m_.mp_.map_voxel_num_; }
  Eigen::Vector3i index(const Eigen::Vector3d& p) {
    Eigen::Vector3i id;
    m_.posToIndex(p, id);
    return id;
  }

  // Counters recomputed from scratch over the stored window.
  int bruteForceMismatches() {
    const int step = m_.inflationStep();
    const Eigen::Vector3i lo = windowMin(), hi = lo + windowSize() - Eigen::Vector3i::Ones();
    int bad = 0;
    for (int x = lo(0); x <= hi(0); ++x)
      for (int y = lo(1); y <= hi(1); ++y)
        for (int z = lo(2); z <= hi(2); ++z) {
          int count = 0;
          for (int i = max(lo(0), x - step); i <= min(hi(0), x + step); ++i)
            for (int j = max(lo(1), y - step); j <= min(hi(1), y + step); ++j)
              for (int k = max(lo(2), z - step); k <= min(hi(2), z + step); ++k)
                if (m_.getOccupancy(Eigen::Vector3i(i, j, k)) == 1) ++count;
          if (count != m_.md_.occupancy_buffer_inflate_[m_.toAddress(Eigen::Vector3i(x, y, z))]) ++bad;
        }
    return bad;
  }

private:
  GridMap& m_;
};

namespace {

int test_count = 0;

// A small map: 10 x 10 x 3 m at 0.1 m, inflation 1 voxel.
std::unique_ptr<GridMap> makeMap(bool raycasting, int max_hits, bool sliding = false) {
  ros::NodeHandle nh("~map" + std::to_string(test_count++));
  nh.setParam("grid_map/resolution", 0.1);
  nh.setParam("grid_map/map_size_x", 10.0);
  nh.setParam("grid_map/map_size_y", 10.0);
  nh.setParam("grid_map/map_size_z", 3.0);
  nh.setParam("grid_map/ground_height", -0.5);
  nh.setParam("grid_map/local_update_range_x", 5.5);
  nh.setParam("grid_map/local_update_range_y", 5.5);
  nh.setParam("grid_map/local_update_range_z", 3.0);
  nh.setParam("grid_map/obstacles_inflation", 0.099);
  nh.setParam("grid_map/skip_pixel", 2);
  nh.setParam("grid_map/p_hit", 0.65);
  nh.setParam("grid_map/p_miss", 0.35);
  nh.setParam("grid_map/p_min", 0.12);
  nh.setParam("grid_map/p_max", 0.90);
  nh.setParam("grid_map/p_occ", 0.80);
  nh.setParam("grid_map/min_ray_length", 0.1);
  nh.setParam("grid_map/max_ray_length", 4.5);
  nh.setParam("grid_map/local_map_margin", 10);
  nh.setParam("grid_map/enable_pointcloud_raycasting", raycasting);
  nh.setParam("grid_map/max_hits_per_update", max_hits);
  nh.setParam("grid_map/sliding_map", sliding);
  nh.setParam("grid_map/sliding_size_x", 6.0);
  nh.setParam("grid_map/sliding_size_y", 6.0);
  nh.setParam("grid_map/sliding_threshold", 0.5);
  std::unique_ptr<GridMap> map(new GridMap);
  map->initMap(nh);
  return map;
}

// A dense patch of wall at x = wall_x, about 16 points per voxel (a lidar
// surface a couple of metres away).
std::vector<Eigen::Vector3d> wall(double wall_x, double y0 = 0.0, double z0 = 0.5) {
  std::vector<Eigen::Vector3d> pts;
  for (double y = y0 - 0.5; y <= y0 + 0.5; y += 0.025)
    for (double z = z0 - 0.3; z <= z0 + 0.3; z += 0.025) pts.emplace_back(wall_x + 0.05, y, z);
  return pts;
}

const Eigen::Vector3d kHome(0.0, 0.0, 0.5);  // sensor at the wall's height

int scansUntilOccupied(GridMap& map, GridMapTestAccess& t, const std::vector<Eigen::Vector3d>& pts,
                       const Eigen::Vector3d& probe) {
  for (int n = 1; n <= 20; ++n) {
    t.scan(pts);
    if (map.getOccupancy(probe) == 1) return n;
  }
  return -1;
}

TEST(GridMap, IncrementalInflationMatchesBruteForce) {
  auto map = makeMap(true, 1, true);
  GridMapTestAccess t(*map);
  std::mt19937 rng(3);
  for (int round = 0; round < 6; ++round) {
    const Eigen::Vector3i lo = t.windowMin(), n = t.windowSize();
    std::uniform_int_distribution<int> x(lo(0), lo(0) + n(0) - 1), y(lo(1), lo(1) + n(1) - 1),
        z(lo(2), lo(2) + n(2) - 1), coin(0, 2);
    for (int i = 0; i < 400; ++i) t.setOccupied(Eigen::Vector3i(x(rng), y(rng), z(rng)), coin(rng) > 0);
    EXPECT_EQ(t.bruteForceMismatches(), 0) << "after random changes, round " << round;
    // Slide in steps that keep part of the window and in one that leaves it.
    t.slide(Eigen::Vector3d(0.8 * round, -0.6 * round, 0.0));
    EXPECT_EQ(t.bruteForceMismatches(), 0) << "after sliding, round " << round;
  }
  t.slide(Eigen::Vector3d(40.0, 40.0, 0.0));
  EXPECT_EQ(t.bruteForceMismatches(), 0);
}

TEST(GridMap, DenseSurfaceIsConfirmedFasterWithHitCounting) {
  const Eigen::Vector3d probe(2.0, 0.0, 0.5);
  auto slow = makeMap(true, 1);
  GridMapTestAccess ts(*slow);
  ts.odom(kHome);
  const int n_slow = scansUntilOccupied(*slow, ts, wall(2.0), probe);

  auto fast = makeMap(true, 6);
  GridMapTestAccess tf(*fast);
  tf.odom(kHome);
  const int n_fast = scansUntilOccupied(*fast, tf, wall(2.0), probe);

  EXPECT_EQ(n_slow, 6);  // logit 0.65 per scan from p_min 0.12 to p_occ 0.80
  EXPECT_EQ(n_fast, 1);
  EXPECT_EQ(fast->getInflateOccupancy(probe + Eigen::Vector3d(-0.1, 0, 0)), 1);  // inflated
}

TEST(GridMap, SingleStrayReturnIsNotAnObstacle) {
  auto map = makeMap(true, 6);
  GridMapTestAccess t(*map);
  t.odom(kHome);
  const Eigen::Vector3d stray(1.55, 1.55, 0.55);  // one return in the voxel's centre
  t.scan({stray});
  EXPECT_EQ(map->getOccupancy(stray), 0);
  EXPECT_EQ(map->getInflateOccupancy(stray), 0);
}

TEST(GridMap, ObstacleThatMovesAwayIsClearedWithItsInflation) {
  auto map = makeMap(true, 6);
  GridMapTestAccess t(*map);
  t.odom(kHome);
  const Eigen::Vector3d near(1.5, 0.0, 0.5);
  t.scan(wall(1.5));
  ASSERT_EQ(map->getOccupancy(near), 1);
  for (int i = 0; i < 3; ++i) t.scan(wall(3.0));  // it moved: rays now pass through
  EXPECT_EQ(map->getOccupancy(near), 0);
  EXPECT_EQ(map->getInflateOccupancy(near), 0);
  EXPECT_EQ(map->getInflateOccupancy(near + Eigen::Vector3d(0.1, 0, 0)), 0);
  EXPECT_EQ(map->getOccupancy(Eigen::Vector3d(3.0, 0.0, 0.5)), 1);
}

TEST(GridMap, RaycastingDoesNotPaintRawPointsBeforeTheUpdate) {
  auto map = makeMap(true, 1);
  GridMapTestAccess t(*map);
  t.odom(kHome);
  t.cloud(wall(2.0));  // no update yet
  EXPECT_EQ(map->getInflateOccupancy(Eigen::Vector3d(2.0, 0.0, 0.5)), 0);
}

TEST(GridMap, ReplacePerScanModeStillWorks) {
  auto map = makeMap(false, 1);
  GridMapTestAccess t(*map);
  t.odom(kHome);
  t.cloud(wall(2.0));
  EXPECT_EQ(map->getInflateOccupancy(Eigen::Vector3d(2.0, 0.0, 0.5)), 1);
  t.cloud(wall(3.0));
  EXPECT_EQ(map->getInflateOccupancy(Eigen::Vector3d(2.0, 0.0, 0.5)), 0);
}

TEST(GridMap, SlidingMapFollowsTheVehicleBeyondTheMapSize) {
  auto map = makeMap(true, 6, true);
  GridMapTestAccess t(*map);
  t.odom(kHome);
  t.scan(wall(1.5));
  ASSERT_EQ(map->getOccupancy(Eigen::Vector3d(1.5, 0.0, 0.5)), 1);
  EXPECT_EQ(map->getInflateOccupancy(Eigen::Vector3d(20.0, 0.0, 0.5)), -1);  // not stored

  // 20 m away: twice the configured map size.
  const Eigen::Vector3d far(20.0, 0.0, 0.5);
  t.odom(far);
  t.scan(wall(21.5));
  EXPECT_EQ(map->getOccupancy(Eigen::Vector3d(21.5, 0.0, 0.5)), 1);
  EXPECT_EQ(map->getInflateOccupancy(Eigen::Vector3d(21.4, 0.0, 0.5)), 1);
  EXPECT_EQ(map->getOccupancy(Eigen::Vector3d(20.8, 0.0, 0.5)), 0);  // seen free
  EXPECT_EQ(map->getOccupancy(Eigen::Vector3d(1.5, 0.0, 0.5)), -1);  // left behind

  // Back home: the old wall was forgotten, nothing stale is left.
  t.odom(kHome);
  t.slide(kHome);
  EXPECT_EQ(map->getOccupancy(Eigen::Vector3d(1.5, 0.0, 0.5)), -1);
  EXPECT_EQ(map->getInflateOccupancy(Eigen::Vector3d(1.5, 0.0, 0.5)), 0);
  EXPECT_EQ(t.bruteForceMismatches(), 0);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_grid_map");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
