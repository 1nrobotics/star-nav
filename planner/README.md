# planner/

The planners, one folder each, and the mission code that gives them goals.
Which robot gets which planner, and what every planner takes and gives:
[docs/PLANNER_INTERFACE.md](../docs/PLANNER_INTERFACE.md).

| Folder | What | Packages | License |
|---|---|---|---|
| `ego/` | EGO-Planner: 3D B-spline planner for the quadrotor | `plan_manage` (ROS package `ego_planner`), `plan_env`, `bspline_opt`, `path_searching`, `traj_utils`, `rosmsg_tcp_bridge` | GPLv3 |
| `cmu/` | CMU's local planner for the legged robots (Lite3, T1), unmodified | `local_planner`, `terrain_analysis` | BSD |
| `exploration_manager/` | Waypoint missions: sends goals to whichever planner runs | `exploration_manager` | MIT |

Each planner's launch file is `star_nav_bringup/launch/planners/<name>.xml`
and its helper nodes, if it has any, are in
`star_nav_bringup/scripts/planners/<name>/` (today only `cmu/`).

`exploration_manager` builds on `ego/plan_env` (the grid map), so it cannot
be built without the EGO folder. Licenses in full:
[docs/LICENSES.md](../docs/LICENSES.md).
