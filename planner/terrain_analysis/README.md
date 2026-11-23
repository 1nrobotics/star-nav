# terrain_analysis

The terrain analysis of Carnegie Mellon University's
[Autonomous Exploration Development Environment](https://www.cmu-exploration.com/)
(Ji Zhang et al.), used here with the CMU local planner
(`exploration_ground.launch planner:=cmu`; see
[docs/GROUND_PLANNING.md](../../docs/GROUND_PLANNING.md)).

- Upstream: <https://github.com/HongbiaoZ/autonomous_exploration_development_environment>,
  branch `noetic`, commit `bf0cba71365271ebff09831a05afd78578150300`,
  directory `src/terrain_analysis`.
- License: BSD, as declared in the upstream `package.xml`. Upstream ships no
  license file for this package.
- `src/terrainAnalysis.cpp` is an unmodified copy. `CMakeLists.txt` and
  `package.xml` were rewritten for this workspace (the unused OpenCV
  dependency is dropped); the upstream launch file is replaced by
  `star_nav_bringup/launch/cmu_planner.xml`.

`terrainAnalysis` keeps the lidar points around the robot (`/registered_scan`,
in the world frame), estimates the ground under them on a 0.2 m grid and
publishes them with their height above it as intensity (`/terrain_map`).
