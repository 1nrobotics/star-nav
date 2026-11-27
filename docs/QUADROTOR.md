# Quadrotor: Development Status

The quadrotor is Star Nav's aerial robot (`robot_type` `aerial`): it flies
the exploration mission with EGO-Planner, in the lightweight simulator and in
MuJoCo. This page records what works, the planner's faults that were found
and what is missing. How to run it: [SIMULATION.md](SIMULATION.md). The
walking robots have their own pages, [QUADRUPED.md](QUADRUPED.md) and
[HUMANOID.md](HUMANOID.md); nothing on this page applies to them: they do
not run EGO.

## Summary

| Area | Status |
|---|---|
| Lightweight simulator (`exploration_sim.launch`) | Working: the quadrotor follows the planner's command exactly; depth camera; collisions are not detected |
| MuJoCo (`exploration_mujoco.launch`) | Working: rigid-body dynamics, motor lag, SE(3) flight controller; contacts are counted |
| Planner | EGO-Planner (`planner/ego/`, GPLv3): position setpoints |
| Obstacle sensor | Forward depth camera, or 360° lidar (`sensor:=lidar`); see [SENSORS.md](SENSORS.md) |
| Mission in the pillar forest | 12/12 waypoints, no contacts, with either sensor |
| Frontier exploration (`explore:=frontier`) | Working in MuJoCo, in the pillar forest, with the lidar: the quadrotor maps the area round its start without waypoints ([below](#frontier-exploration-explorefrontier)) |
| Localisation | The simulator's pose |
| Hardware | `hardware/vk180` branch ([HARDWARE.md](HARDWARE.md)) |
| Tests | Local: `scripts/run_local_tests.sh aerial`. Docker: `scripts/run_smoke_test.sh`, `scripts/run_mujoco_smoke_test.sh` |

## Architecture

The quadrotor is an aerial robot, so it flies with EGO-Planner
([PLANNER_INTERFACE.md](PLANNER_INTERFACE.md)):

```
mission waypoints (exploration_manager)
  -> ego_planner_node: a B-spline trajectory through its voxel map
  -> traj_server: position setpoints (position, velocity, acceleration, yaw)
  lightweight simulator: the quadrotor takes the setpoint as its pose
  MuJoCo, over the ZeroMQ bridge:
  -> SE(3) flight controller: setpoint -> 4 rotor thrusts

depth image and camera pose, or lidar cloud (world frame) -> EGO's voxel map
```

| Parameter | Value |
|---|---|
| `max_vel`, `max_acc` | 1.5 m/s, 3.0 m/s² |
| Flight height (`altitude`) | 1.0 m |
| `obstacles_inflation` | 0.2 m in MuJoCo |
| Waypoint tolerance | 0.5 m |
| Waypoint timeout | 30 s |

The map (ray casting, hits per update, sliding window) is described in
[SENSORS.md](SENSORS.md); the simulator in
[SIM_ARCHITECTURE.md](SIM_ARCHITECTURE.md).

### Frontier exploration (`explore:=frontier`)

Without preset waypoints, in MuJoCo (`exploration_mujoco.launch`):
`star_nav_bringup/scripts/frontier_explorer.py` takes the place of
`exploration_manager`. From the lidar scans it keeps a 2D map of what stands
in the way at flight height, sends EGO the nearest border between seen free
space and unseen space as its next goal (`/move_base_simple/goal`, at the
flight height), and the start as the last goal when no border is left. The
node, its map and its rules are those of the walking robots
([QUADRUPED.md](QUADRUPED.md#frontier-exploration-explorefrontier)); the
quadrotor gives it other numbers:

| Parameter | Value |
|---|---|
| Sensor | The lidar, whatever `sensor` says: the map is made from lidar scans |
| Goal height (`altitude`) | 1.0 m |
| Obstacle (`robot_height`) | What rises from the ground into the 0.5 m above the flight height (1.5 m) |
| Goals' distance from obstacles (`clearance`) | `obstacles_inflation` + 0.3 m: 0.5 m |
| Area | The exploration area plus `frontier_margin` (1.5 m) on every side: 12 m x 12 m by default |

The map is on `/frontier_explorer/map` (`nav_msgs/OccupancyGrid`), separate
from EGO's voxel map. EGO flies straight to each goal through its own map;
nothing plans a route (see [Limitations](#limitations)).

## Measured

Frontier exploration in MuJoCo, seed 1 (2026-11-27; times are simulated,
from the trigger to `FINISH`):

| Check | Result |
|---|---|
| `mujoco_frontier_smoke.test` (12 m x 12 m), 10 runs (`run_local_tests.sh --repeat 10 mujoco-frontier`) | 10 passed: 4 to 8 frontiers, 99.6 to 99.7 % of the area known, 22 to 27 s, 26 to 30 m flown, home within 0.06 to 0.30 m, 0 contacts |
| 24 m x 24 m (`exploration_width:=21 exploration_height:=21`), one run | 43 frontiers, 99.7 % of the area known, 64 s, 82 m flown, home within 0.18 m, 0 contacts |

## Tests

The `aerial` group of `scripts/run_local_tests.sh`: EGO's map unit tests,
the mission in the lightweight simulator, the mission in MuJoCo with the
depth camera and with the lidar, and frontier exploration in MuJoCo
(`mujoco_frontier_smoke.test`). What each must show, and how a failure that
does not happen every time is judged: [TESTING.md](TESTING.md).

## Planner faults

Faults of EGO-Planner found while testing. They concern the quadrotor only.

**Fixed**

- **Waypoint callback** (`ego_replan_fsm.cpp`): it busy-waited for
  `EXEC_TRAJ`. If the trajectory ended during that wait, the planner hung and
  ignored all further waypoints. This was behind lidar missions that failed
  now and then (2 of 4 runs). It now triggers from `WAIT_TARGET` or replans
  from `EXEC_TRAJ` directly; 4/4 runs pass.

- **Replanning from a trajectory that is ending** (`planner_manager.cpp`,
  `reboundReplan`): the planner samples the rest of its previous trajectory
  as the start of the new one, and loops until it has 7 points. With less
  than one sampling step (0.4 s) of the trajectory left and the target
  within 0.3 m of its end, there is one sample and no length: the loop never
  ended, and read past its arrays. The node has one thread, so commands
  stopped and every later waypoint timed out; `ego_planner_node` used a whole
  processor core and printed nothing. It happened at the mission's first
  corner, where three waypoints 0.5 to 1 m apart are reached within about a
  second: 4 of 108 runs of the lidar mission up to that corner, and 3 of 39
  when measured again (2026-11-26). The planner now starts from a polynomial
  in that case (and when the previous trajectory has already ended). With
  the fix, 80 of 80 runs passed the corner; in 79 earlier runs under a
  debugger the case came up twice and the mission went on.

**Open**

None known.

## Limitations

- **Local planner only.** A goal behind a wall may leave the quadrotor
  stuck; there is no route planner as for the walking robots.
- **Frontier exploration without a route planner.** A frontier is given to
  EGO as it is. In the pillar forest that is enough; a frontier behind a
  wall is given up after 60 s (`goal_timeout`), or after 6 s of standing
  still, and the area behind it stays unmapped. It needs the lidar, and it
  runs in MuJoCo only.
- **The lightweight simulator detects no collisions.** Only the MuJoCo
  missions check contacts.
- **Tested in the pillar forest only**, not in the house.
- **Simulator pose.** No odometry or SLAM is in the loop on `main`.
