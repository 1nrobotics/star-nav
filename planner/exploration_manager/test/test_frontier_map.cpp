#include <gtest/gtest.h>

#include <functional>

#include "exploration_manager/frontier_map.h"

using exploration_manager::FrontierMap;

namespace {

// A 10 x 10 m area around the origin.
FrontierMap makeMap(double inflation = 0.3) {
  FrontierMap m;
  FrontierMap::Params p;
  p.inflation = inflation;
  m.init(Eigen::Vector2d(-5, -5), Eigen::Vector2d(5, 5), p);
  return m;
}

// Merges a 0.1 m grid over [-7, 7]^2 whose cells are given by f(x, y):
// -1 unknown, 0 free, 100 occupied.
void observe(FrontierMap& m, const std::function<int(double, double)>& f) {
  const int n = 140;
  std::vector<int8_t> data(n * n);
  for (int y = 0; y < n; ++y)
    for (int x = 0; x < n; ++x)
      data[x + y * n] = f(-7.0 + (x + 0.5) * 0.1, -7.0 + (y + 0.5) * 0.1);
  m.merge(Eigen::Vector2d(-7, -7), 0.1, n, n, data);
}

bool inArea(double x, double y) {
  return std::abs(x) < 5 && std::abs(y) < 5;
}

}  // namespace

TEST(FrontierMap, UnobservedRingAroundTheRobotIsAFrontier) {
  FrontierMap m = makeMap();
  observe(m, [](double x, double y) { return std::hypot(x, y) < 2.0 ? 0 : -1; });
  m.update(Eigen::Vector2d(0, 0));
  ASSERT_TRUE(m.robotInFreeSpace());
  ASSERT_GE(m.frontiers().size(), 4u);  // a 12.6 m ring split into <= 3 m parts
  for (const auto& f : m.frontiers()) {
    EXPECT_NEAR(f.goal.norm(), 1.95, 0.15);
    EXPECT_LT(f.distance, 2.6);
  }
  EXPECT_NEAR(m.coverage(), M_PI * 4 / 100, 0.01);
}

TEST(FrontierMap, FullyObservedAreaHasNoFrontier) {
  FrontierMap m = makeMap();
  // Unknown outside the area does not count.
  observe(m, [](double x, double y) { return inArea(x, y) ? 0 : -1; });
  m.update(Eigen::Vector2d(0, 0));
  EXPECT_TRUE(m.frontiers().empty());
  EXPECT_DOUBLE_EQ(m.coverage(), 1.0);
}

TEST(FrontierMap, UnknownBehindAClosedWallIsUnreachable) {
  FrontierMap m = makeMap();
  observe(m, [](double x, double y) {
    if (std::abs(x - 1.0) < 0.1)
      return 100;
    return x < 1.0 ? 0 : -1;
  });
  m.update(Eigen::Vector2d(-2, 0));
  EXPECT_TRUE(m.frontiers().empty());
  EXPECT_EQ(m.distance(Eigen::Vector2d(3, 0)), FrontierMap::kUnreachable);
}

TEST(FrontierMap, GapNarrowerThanTheRobotIsNotPassable) {
  for (double gap : {0.4, 1.2}) {
    FrontierMap m = makeMap();
    observe(m, [gap](double x, double y) {
      if (std::abs(x - 1.0) < 0.1)
        return std::abs(y) < gap / 2 ? 0 : 100;
      return x < 1.2 ? 0 : -1;
    });
    m.update(Eigen::Vector2d(-2, 0));
    // 0.3 m inflation: a 0.4 m gap is closed, a 1.2 m gap is open.
    EXPECT_EQ(m.frontiers().empty(), gap < 0.6) << "gap " << gap;
  }
}

TEST(FrontierMap, RobotInsideTheInflationEscapesToFreeSpace) {
  FrontierMap m = makeMap();
  observe(m, [](double x, double y) {
    if (std::hypot(x - 0.25, y) < 0.1)
      return 100;  // an obstacle 0.25 m from the robot
    return std::hypot(x, y) < 2.0 ? 0 : -1;
  });
  m.update(Eigen::Vector2d(0, 0));
  EXPECT_TRUE(m.robotInFreeSpace());
  EXPECT_FALSE(m.frontiers().empty());
}

TEST(FrontierMap, IsolatedUnknownHolesAreNotFrontiers) {
  FrontierMap m = makeMap();
  observe(m, [](double x, double y) {
    if (!inArea(x, y))
      return -1;
    // Single unknown cells every metre.
    const double fx = x - std::floor(x), fy = y - std::floor(y);
    return fx < 0.1 && fy < 0.1 ? -1 : 0;
  });
  m.update(Eigen::Vector2d(0.5, 0.5));
  EXPECT_TRUE(m.frontiers().empty());
}

TEST(FrontierMap, BlockedGoalsAreSkipped) {
  FrontierMap m = makeMap();
  // Observed except a 2 m wide unknown patch in one corner.
  observe(m, [](double x, double y) { return inArea(x, y) && !(x > 3 && y > 3) ? 0 : -1; });
  m.update(Eigen::Vector2d(0, 0));
  ASSERT_FALSE(m.frontiers().empty());
  const Eigen::Vector2d goal = m.frontiers()[0].goal;
  m.update(Eigen::Vector2d(0, 0), {goal}, 3.0);
  EXPECT_TRUE(m.frontiers().empty());
}

TEST(FrontierMap, ObservationsAreKeptAndUpdated) {
  FrontierMap m = makeMap();
  observe(m, [](double x, double y) { return std::abs(x) < 1 && std::abs(y) < 1 ? 100 : 0; });
  // A later slice: unknown there now (out of the sensor window) except one
  // cell seen free, which does not clear the obstacle.
  observe(m, [](double x, double y) { return std::hypot(x - 0.55, y - 0.55) < 0.05 ? 0 : -1; });
  const int c = (int)std::floor((0.05 - m.origin()(0)) / m.resolution());
  EXPECT_EQ(m.cell(c, c), FrontierMap::OCCUPIED);
  m.update(Eigen::Vector2d(3, 3));
  EXPECT_EQ(m.distance(Eigen::Vector2d(0, 0)), FrontierMap::kUnreachable);
  EXPECT_LT(m.distance(Eigen::Vector2d(-3, -3)), FrontierMap::kUnreachable);
}

TEST(FrontierMap, ObstaclesStayWhenLaterSeenFree) {
  FrontierMap m = makeMap();
  observe(m, [](double x, double y) { return std::hypot(x - 1.0, y) < 0.1 ? 100 : 0; });
  observe(m, [](double x, double y) { return 0; });  // the thin obstacle dropped out
  m.update(Eigen::Vector2d(0, 0));
  EXPECT_EQ(m.distance(Eigen::Vector2d(1.0, 0.0)), FrontierMap::kUnreachable);
}

TEST(FrontierMap, PathGoesAroundObstaclesInLineOfSightLegs) {
  FrontierMap m = makeMap();
  // A wall from y = -3 to 3 at x = 0, robot west of it, goal east.
  auto wall = [](double x, double y) { return std::abs(x) < 0.1 && std::abs(y) < 3.0; };
  observe(m, [&](double x, double y) { return wall(x, y) ? 100 : inArea(x, y) ? 0 : -1; });
  m.update(Eigen::Vector2d(-2, 0));
  const auto route = m.path(Eigen::Vector2d(2, 0), 3.0);
  ASSERT_FALSE(route.empty());
  EXPECT_LT((route.back() - Eigen::Vector2d(2, 0)).norm(), 0.1);
  Eigen::Vector2d prev(-2, 0);
  for (const auto& w : route) {
    EXPECT_LE((w - prev).norm(), 3.0 + 0.15);
    // No leg crosses the wall (it would pass x = 0 at |y| < 3.3).
    if ((prev(0) < 0) != (w(0) < 0)) {
      const double y = prev(1) + (w(1) - prev(1)) * (0 - prev(0)) / (w(0) - prev(0));
      EXPECT_GT(std::abs(y), 3.3);
    }
    prev = w;
  }
  EXPECT_TRUE(m.path(Eigen::Vector2d(20, 20), 3.0).empty());  // outside the map
}

TEST(FrontierMap, PlannerInflationBlocksButCanBeLeft) {
  FrontierMap m = makeMap(0.0);
  observe(m, [](double x, double y) {
    if (std::hypot(x - 0.3, y) < 0.1)
      return 100;  // an obstacle 0.3 m from the robot
    if (std::hypot(x - 0.3, y) < 0.5)
      return 50;  // the planner's inflation around it, robot inside
    return std::hypot(x, y) < 2.0 ? 0 : -1;
  });
  m.update(Eigen::Vector2d(0, 0));
  EXPECT_TRUE(m.robotInFreeSpace());
  EXPECT_FALSE(m.frontiers().empty());
  EXPECT_EQ(m.distance(Eigen::Vector2d(0.5, 0)), FrontierMap::kUnreachable);  // blocked
  EXPECT_LT(m.distance(Eigen::Vector2d(-1, 0)), FrontierMap::kUnreachable);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
