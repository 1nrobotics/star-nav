# Planner Interface

Star Nav moves three robots with two planners: the quadrotor flies with
EGO-Planner; the Lite3 quadruped and the T1 humanoid walk with the ground
version of CMU's local planner. This page describes the one interface both planners sit behind:
what a planner takes and gives, how a robot gets its planner, where each
planner's files are, and how to add a planner or a kind of robot. For what
each planner does inside, see [GROUND_PLANNING.md](GROUND_PLANNING.md).
Written November 2026.

## Why

Before this, the planner was chosen in three places.
`planning_stack.xml` started EGO unless told `planner:=cmu`;
`exploration_ground.launch` started the CMU planner and the terrain filter,
and sent the simulator's lidar to a different topic for each case; the
quadrotor's launch files repeated EGO's sensor options. Adding a planner or
a kind of robot meant editing all of them.

Now there is one place, `star_nav_bringup/launch/planner.xml`, and one rule:
**a robot says what type it is; the type decides the planner.**

| `robot_type` | Robots | Planner |
|---|---|---|
| `aerial` | quadrotor (lightweight simulator, MuJoCo) | `ego` |
| `legged` | Lite3 quadruped, T1 humanoid | `cmu` |

So a drone flies with EGO and a legged robot walks with the CMU planner.
This is a choice, not a limit of either planner's family: CMU publishes its
local planner in a ground version and an aerial version (see
[CMU's two planners](#cmus-two-planners)). Star Nav has the ground version,
as the `cmu` backend, and flies the quadrotor with EGO.
`planner:=auto` (the default) takes the type's planner; `planner:=<name>`
asks for one by name, and works where the table has the pair. A pair that is
not in the table stops the launch with an error naming it:

- `aerial:cmu`: the `cmu` backend is the ground version of CMU's planner.
  It plans in 2D and sends a ground vehicle's velocity. CMU's aerial version
  is not in this repository.
- `legged:ego`: EGO plans a quadrotor's flight. It moved the walking robots
  before the CMU planner was added, at a fixed height and through a filter
  that turned the lidar's scans into obstacle columns
  ([GROUND_PLANNING.md](GROUND_PLANNING.md) has the comparison). That path
  and its terrain filter were removed: each kind of robot has one planner,
  the one made for it.

## Structure

```
exploration_sim.launch      robot_type aerial ┐
exploration_mujoco.launch   robot_type aerial ├─► planning_stack.xml
exploration_<robot>.launch  robot_type legged │     ├─ planner.xml          the interface
  └ exploration_ground.launch                 ┘     │    ├─ planners/ego.xml   EGO planner, trajectory
                                                    │    │                     server
                                                    │    └─ planners/cmu.xml   terrain analysis, local
                                                    │                          planner, path follower,
                                                    │                          waypoint router
                                                    ├─ exploration manager (waypoints)
                                                    └─ RViz
```

- **`planner.xml`** is the interface. Its arguments describe the robot, the
  world and the sensor, not a planner. It resolves `robot_type` and
  `planner` to a backend and includes `planners/<backend>.xml`. It names no
  planner outside the table.
- **`planners/<backend>.xml`** is everything one planner needs, including
  what it puts in front of itself. Every backend is included the same way,
  with all of `planner.xml`'s arguments; it declares the ones it uses and
  says which kind of command it gives. The waypoint router, for instance,
  belongs to the CMU backend.
- **The simulator's launch file** no longer knows which planner runs. It
  publishes the raw sensor data on fixed topics and accepts both kinds of
  command.

## Where each planner's files are

Each planner's code is in its own folder, so one can be read or replaced
without touching the other:

| | `ego` | `cmu` | Shared |
|---|---|---|---|
| Source packages | `planner/ego/` (`plan_manage`, `plan_env`, `bspline_opt`, `path_searching`, `traj_utils`, `rosmsg_tcp_bridge`) | `planner/cmu/` (`local_planner`, `terrain_analysis`) | `planner/exploration_manager` (waypoint missions) |
| Launch | `star_nav_bringup/launch/planners/ego.xml` | `star_nav_bringup/launch/planners/cmu.xml` | `launch/planner.xml`, `launch/planning_stack.xml` |
| Helper nodes | none | `star_nav_bringup/scripts/planners/cmu/cmu_waypoint_router.py` | `scripts/frontier_explorer.py`, `frontier_map.py` |
| License | GPLv3 | BSD (the router is this project's) | |

Package names did not change with the folders (`ego_planner`, `plan_env`,
`local_planner`, ...), so `rosrun`, `roslaunch` and `find_package` work as
before. Two things still cross the line: `exploration_manager` builds on
`plan_env` of the EGO folder (its waypoint generator includes the grid
map's header), and each simulator launch file and RViz configuration
names its planner's display topics (EGO's trajectory markers for the
quadrotor, the CMU planner's paths and terrain map for the walking robots).

## The contract

What a backend takes and gives. Everything outside the planner (the
simulator's bridge, the exploration manager, frontier exploration, the
tests) relies on this and nothing else.

### Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | Where to go, world frame. A 2D planner ignores z. A new goal replaces the old one. |
| in | `/drone_<id>_<odom_topic>` | `nav_msgs/Odometry` | The robot's pose and velocity, world frame. |
| in | `/drone_<id>_<cloud_topic>` | `sensor_msgs/PointCloud2` | Lidar returns in the world frame (`sensor:=lidar`). |
| in | `/drone_<id>_<depth_topic>`, `/drone_<id>_<camera_pose_topic>` | `sensor_msgs/Image`, `geometry_msgs/PoseStamped` | Depth image and the camera's pose (`sensor:=depth`). |
| out | `/drone_<id>_planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | Command kind `position`: setpoints (position, velocity, acceleration, yaw). |
| out | `/cmd_vel` | `geometry_msgs/Twist` | Command kind `velocity`: body frame, forward and turn rate. |

A backend gives one kind of command, and the simulator's bridge listens for
both: the quadrotor follows position setpoints, a walking robot's policy a
velocity ([SIM_ARCHITECTURE.md](SIM_ARCHITECTURE.md)).

The `drone_<id>_` prefix is EGO-Planner's naming, kept for every robot so
that RViz configurations and tests work for all of them.

### Parameters

What was resolved is on the parameter server, for tests and tools that need
to know:

| Parameter | Values | Set by |
|---|---|---|
| `/planner/backend` | `ego`, `cmu` | `planner.xml` |
| `/planner/robot_type` | `aerial`, `legged` | `planner.xml` |
| `/planner/command` | `position`, `velocity` | the backend |

### Arguments of `planner.xml`

| Group | Arguments | Meaning |
|---|---|---|
| Choice | `robot_type`, `planner` | See the table above. |
| Topics | `drone_id`, `odom_topic`, `cloud_topic`, `depth_topic`, `camera_pose_topic` | Names without the `/drone_<id>_` prefix. |
| World | `map_size_x`, `map_size_y`, `map_size_z` | Metres. |
| Robot | `max_vel`, `max_acc`, `obstacles_inflation` | Speed and acceleration limits; clearance to keep (at least the robot's radius). |
| Robot (ground) | `robot_height` | Higher things are overhead, not obstacles. |
| Sensor | `sensor`, `lidar_offset`, `cx`, `cy`, `fx`, `fy` | `depth` or `lidar`; lidar position in the body frame; camera intrinsics. |
| EGO options | `lidar_raycasting`, `lidar_max_hits`, `sliding_map`, `sliding_size`, `show_occ_time`, `planning_horizon`, `obj_num`, `visualization_min_height`, `visualization_max_height` | Used by `planners/ego.xml` only. |
| CMU options | `cmu_obstacle_height`, `cmu_path_scale`, `cmu_min_path_scale` | Used by `planners/cmu.xml` only. |

The first six groups mean the same to every backend. A backend's own
options are named for it and have defaults, so a launch file that does not
care never sets them.

A backend receives all of these (`pass_all_args`) and declares, without a
default, the ones it uses: a missing one stops the launch. What only one
backend needs to know stays inside its file (the CMU backend's turn rate and
path folder, EGO's flight type).

## The two backends

| | `ego` | `cmu` (CMU's ground version) |
|---|---|---|
| Plans in | 3D (B-spline optimization on a voxel map) | 2D (one of 343 pre-computed paths on a terrain map) |
| Command | position setpoints | velocity |
| Sensors | depth camera or lidar | lidar only |
| Robots | aerial | legged |
| Ramps and steps | not applicable | ground, by terrain analysis |
| Goal behind a wall | may get stuck: local planner only | routed by the waypoint router (A*) |
| Frontier exploration (`explore:=frontier`) | no | yes |
| Extra files | none | path set, `scripts/fetch_cmu_paths.sh` |
| License | GPLv3 | BSD |

What the EGO backend derives from the common arguments (it was spread over
the launch files before):

| Setting | Value |
|---|---|
| Map's input | the depth image and camera pose, or `cloud_topic` with `sensor:=lidar` |
| Ray casting of the cloud | `sensor:=lidar` and `lidar_raycasting:=true` |
| Hits per map update | `lidar_max_hits` with the lidar, 1 with the depth camera |

### CMU's two planners

CMU publishes its local planner twice. Both pick a path from a pre-computed
set and follow it; they differ in what the paths are and what is commanded:

| | Ground version | Aerial version |
|---|---|---|
| Repository | [autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment) | [aerial_navigation_development_environment](https://github.com/caochao39/aerial_navigation_development_environment) |
| Paths | 343 in 7 groups, in the plane | 4,375 in 25 groups, in 3D |
| Obstacles | lidar points after terrain analysis (height above the ground) | a range sensor's point cloud |
| Command | velocity (forward, turn rate) | attitude (roll, pitch, yaw rate, vertical speed) |
| License | BSD (each `package.xml`) | CMU non-commercial |
| In Star Nav | yes: `planner/cmu/`, backend `cmu` | no |

The aerial version's numbers are read from its code (branch
`melodic-noetic`, commit `0dcc4b1`); it was not built or run here. For the
quadrotor Star Nav chose EGO-Planner, which the project started with, and
did not compare the two. The aerial version would be one more backend for
`aerial` (see below). Its command is a third kind, an attitude command, and
its license does not allow commercial use, which would have to be recorded
in [LICENSES.md](LICENSES.md).

## Adding a planner

1. Put its packages in `planner/<name>/` and its helper nodes in
   `star_nav_bringup/scripts/planners/<name>/`.
2. Write `launch/planners/<name>.xml`. Declare the common arguments it
   uses; subscribe to the goal, the odometry and the sensor topics above;
   publish one of the two kinds of command and set `/planner/command`
   to it. Refuse what it cannot do (the CMU backend refuses
   `sensor:=depth`).
3. In `planner.xml`: add the rows of the `backend` table for the robot
   types it can move. There is no include to add: the backend's file is
   found by its name. Options a robot's launch file must be able to set go
   in `planner.xml` too, prefixed with the planner's name.
4. Add it to `PLANNERS` and `COMMAND` in
   `star_nav_bringup/test/test_planner_interface.py` and to the smoke tests
   of the robots it moves.
5. Record its license in [LICENSES.md](LICENSES.md).

## Adding a kind of robot

A robot type is a name in the `backend` table of `planner.xml` with its
default planner. A wheeled robot, for instance, would add `wheeled:auto` and
`wheeled:cmu`, both `cmu`, and a launch file that sets `robot_type` to
`wheeled` for `exploration_ground.launch`, as `exploration_t1.launch` sets
`legged`. A second planner for a type is one more row (`legged:<name>`),
chosen with `planner:=<name>`. The simulator side of a new robot is in
[SIM_ARCHITECTURE.md](SIM_ARCHITECTURE.md).

## Tests

- `star_nav_bringup/test/test_planner_interface.py` loads every robot's
  launch file the way `roslaunch` does, without starting anything, and
  checks which planner would run, that a pair outside the table is refused
  (and the CMU planner with the depth camera), that `launch/planners/` holds
  exactly the table's backends, and that the planner is wired to the robot's
  topics. It runs in a second:
  the `launch` step of `./scripts/run_local_tests.sh`.
- The smoke tests run each pair for real: `sim_smoke.test` and
  `mujoco_smoke.test` (aerial, ego), `lite3_smoke.test`, `t1_smoke.test`
  and `frontier_smoke.test` (legged, cmu).

## Limits and next steps

- **The command is still one of two kinds.** A planner that outputs a path
  (for a robot's own controller) would be a third.
- **No common status.** Whether a goal was reached, is blocked or was given
  up is judged by whoever sent it, from the odometry (the exploration
  manager, the frontier explorer). A `/planner/status` topic every backend
  publishes would let them share that, and is what a "go to" command needs
  to return ([issue #58](https://github.com/1nrobotics/star-nav/issues/58)).
- **Paths for display differ**: EGO publishes markers, the CMU planner
  `nav_msgs/Path` in the vehicle frame. The simulator's bridge takes both.
- **The table is in a launch file.** `roslaunch` cannot read it from a
  configuration file, so a new robot type is an edit of `planner.xml`.
- **`exploration_manager` needs the EGO folder to build** (`plan_env`), so
  the CMU planner cannot yet be built without the GPLv3 packages
  ([LICENSES.md](LICENSES.md)).
