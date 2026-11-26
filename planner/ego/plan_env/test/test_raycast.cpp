// RayCaster must visit every cell the segment passes through, in order from
// start to end, and stop at the end cell. grid_map clears free space with it,
// so a skipped cell is a stale obstacle that is never removed.
#include <gtest/gtest.h>
#include <plan_env/raycast.h>

#include <cmath>
#include <random>
#include <set>
#include <tuple>
#include <vector>

namespace
{
using Cell = std::tuple<int, int, int>;

Cell cellOf(const Eigen::Vector3d &p)
{
  return Cell((int)std::floor(p.x()), (int)std::floor(p.y()), (int)std::floor(p.z()));
}

std::vector<Cell> walk(const Eigen::Vector3d &start, const Eigen::Vector3d &end)
{
  std::vector<Cell> cells;
  RayCaster caster;
  Eigen::Vector3d p;
  if (!caster.setInput(start, end))
    return cells;
  while (caster.step(p) && cells.size() < 10000)
    cells.push_back(cellOf(p));
  return cells;
}

void expectCovers(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
  std::vector<Cell> cells = walk(a, b);
  ASSERT_LT(cells.size(), 10000u) << "walk did not terminate";
  std::set<Cell> walked(cells.begin(), cells.end());
  walked.insert(cellOf(b));  // step() stops before reporting the end cell
  for (int i = 0; i <= 5000; ++i)
  {
    Eigen::Vector3d q = a + (b - a) * (i / 5000.0);
    ASSERT_TRUE(walked.count(cellOf(q)))
        << "cell of (" << q.transpose() << ") skipped on " << a.transpose() << " -> " << b.transpose();
  }
  // Adjacent cells only: one axis changes by one per step.
  for (size_t i = 1; i < cells.size(); ++i)
  {
    int d = std::abs(std::get<0>(cells[i]) - std::get<0>(cells[i - 1])) +
            std::abs(std::get<1>(cells[i]) - std::get<1>(cells[i - 1])) +
            std::abs(std::get<2>(cells[i]) - std::get<2>(cells[i - 1]));
    ASSERT_EQ(d, 1);
  }
}
}  // namespace

// A nearly horizontal lidar beam that crosses one z boundary late: the old
// stepping used the cell index difference and left the z cell too early.
TEST(RayCaster, ShallowRayCrossesBoundaryWhereTheSegmentDoes)
{
  expectCovers(Eigen::Vector3d(-26.5, 13.7, 11.27), Eigen::Vector3d(0.0, 0.0, 10.7));
}

TEST(RayCaster, AxisAlignedAndDegenerateSegments)
{
  expectCovers(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(7.5, 0.5, 0.5));
  expectCovers(Eigen::Vector3d(0.5, 3.2, 0.5), Eigen::Vector3d(0.5, -4.9, 0.5));
  expectCovers(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(4.0, 2.0, 3.0));  // on cell boundaries
  EXPECT_TRUE(walk(Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.8, 0.7, 0.9)).empty());
}

TEST(RayCaster, RandomSegmentsVisitEveryCell)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> pos(-30.0, 30.0), len(0.5, 45.0), unit(-1.0, 1.0);
  for (int k = 0; k < 2000; ++k)
  {
    Eigen::Vector3d a(pos(rng), pos(rng), pos(rng) / 6.0);
    Eigen::Vector3d dir(unit(rng), unit(rng), unit(rng));
    if (dir.norm() < 1e-3)
      continue;
    Eigen::Vector3d b = a + dir.normalized() * len(rng);
    if (k % 5 == 0)
      b.z() = a.z();
    expectCovers(a, b);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
