# local_planner

The local planner and path follower of Carnegie Mellon University's
[Autonomous Exploration Development Environment](https://www.cmu-exploration.com/)
(Ji Zhang et al.), used here to steer the ground robots
(`planner:=cmu`; see
[docs/GROUND_PLANNING.md](../../../docs/GROUND_PLANNING.md)).

- Upstream: <https://github.com/HongbiaoZ/autonomous_exploration_development_environment>,
  branch `noetic`, commit `bf0cba71365271ebff09831a05afd78578150300`,
  directory `src/local_planner`.
- License: BSD, as declared in the upstream `package.xml`. Upstream ships no
  license file for this package.
- `src/localPlanner.cpp` and `src/pathFollower.cpp` are unmodified copies.
  `CMakeLists.txt` and `package.xml` were rewritten for this workspace; the
  upstream launch file is replaced by
  `star_nav_bringup/launch/planners/cmu.xml`.
- The pre-computed path set (`paths/`, 29 MB) is not in this repository:
  `scripts/fetch_cmu_paths.sh` downloads it from the commit above into
  `~/.cache/star_nav/cmu_paths` and checks it.

This is CMU's planner for ground vehicles. CMU's planner for aerial vehicles
is a different repository
([aerial_navigation_development_environment](https://github.com/caochao39/aerial_navigation_development_environment),
CMU non-commercial license) and is not used here: the quadrotor flies with
EGO-Planner ([docs/PLANNER_INTERFACE.md](../../../docs/PLANNER_INTERFACE.md#cmus-two-planners)).

`localPlanner` picks, from the pre-computed paths, the group that is clear of
obstacles and best aligned with the waypoint (`/way_point`), and publishes it
in the vehicle frame (`/path`). `pathFollower` follows it and publishes a
body-frame velocity (`/cmd_vel`). Both plan in 2D.
