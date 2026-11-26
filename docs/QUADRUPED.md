# Quadruped (Lite3): Development Status

Status of the walking-robot path on the `feature/quadruped` branch: what
works, how it fits together, what was measured, and what is missing. How to
run it: [SIMULATION.md](SIMULATION.md#mujoco-simulator) and
[mujoco_sim/README.md](../mujoco_sim/README.md#lite3-quadruped).

## Summary

| Area | Status |
|---|---|
| Lite3 in MuJoCo, upstream walking policy | Working |
| CMU local planner driving the robot (default) | Working: terrain analysis, local planner, path follower; velocity commands |
| EGO planner driving the robot (`planner:=ego`) | Working: the drone's stack at a fixed height |
| Forest, lidar (recommended) or depth camera | 12/12 waypoints, no contacts |
| House with the lidar: furniture, 12° ramp, 0.21 m platform, 7 cm steps | 12/12 waypoints, no contacts |
| House with the depth camera, or with `terrain:=false` | Not supported: see Limitations |
| Terrain filter (ramps and low steps are walkable) | Working, static worlds only |
| Localisation | Simulator ground truth |
| Steps above 8 cm | Not supported (blind policy) |
| Hardware | None |
| Tests | Local: `scripts/run_local_tests.sh lite3` (needs the downloaded assets) |

## Architecture

With the CMU local planner (`planner:=cmu`, the default; lidar only):

```
mission waypoints (exploration_manager)
  -> cmu_waypoint_router.py: route around the walls (A* on the terrain map),
     a waypoint up to 3 m along it
  -> localPlanner: the pre-computed path group that is free and best aligned
  -> pathFollower: body-frame velocity (forward, turn), 50 Hz
  ==== ZeroMQ bridge ==== MuJoCo
  -> tracking.Holonomic.limit: speed and turn-rate limits
  -> walking policy (ONNX, 83 Hz): command -> 12 joint targets
  -> PD (500 Hz): joint targets -> torques

lidar (world frame) -> terrainAnalysis: height above the ground -> localPlanner
```

With the EGO planner (`planner:=ego`):

```
mission waypoints (exploration_manager)
  -> EGO-Planner: B-spline at a fixed height (0.34 m, the torso)
  -> traj_server: position, velocity, yaw setpoints (the same as for the drone)
  ==== ZeroMQ bridge ==== MuJoCo
  -> tracking.Holonomic: setpoint -> body-frame (forward, sideways, turn)
  -> walking policy (ONNX, 83 Hz): command -> 12 joint targets
  -> PD (500 Hz): joint targets -> torques

lidar (world frame) -> terrain_filter.py -> grid_map -> EGO
```

### Control (`mujoco_sim/star_nav_mujoco/robots/lite3.py`)

- **Model and policy**: DeepRobotics' Lite3 MJCF
  ([deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model))
  and the PPO velocity policy from
  [Lite3_rl_deploy](https://github.com/DeepRoboticsLab/Lite3_rl_deploy), both
  BSD-3-Clause. They are downloaded at pinned commits with a SHA-256 check
  (`star-nav-mujoco --fetch lite3`), not committed. We did not train anything.
- **Policy**: 45 observations (body angular velocity, projected gravity,
  command, joint positions and velocities, last action). The command is limited
  to ±0.8. The policy outputs joint offsets from the standing pose. It runs
  every 6 physics steps (2 ms each, about 83 Hz).
- **PD**: kp 30, kd 1, ±30 N·m, every physics step. These match the upstream
  deployment.
- **Tracker**:
  - velocity = setpoint velocity + 1.5 × position error, capped at 0.8 m/s,
    rotated into the body frame;
  - turn rate = 1.5 × yaw error, capped at 0.8 rad/s;
  - stands still within 3 cm and 3° of a resting setpoint.

  The robot walks in any direction, so it does not turn to face its path
  before moving.
- **Safety in the simulator**: a fall is logged when the torso drops below half
  the standing height. Contacts with the ground count as walking, not
  collisions; so do feet and lower legs on `walk/` geoms (ramp, platform,
  steps).

### Planning

- **CMU local planner** (default): see
  [GROUND_PLANNING.md](GROUND_PLANNING.md#10-the-cmu-planner-in-star-nav)
  for how it is wired and its parameters. The rest of this section is the
  EGO path (`planner:=ego`).
- **Planner**: EGO-Planner, the same as for the drone, at a fixed altitude of
  0.34 m. The parameters are in `star_nav_bringup/launch/exploration_lite3.launch`
  (the robot's sizes) and `exploration_ground.launch` (the rest):

  | Parameter | Value |
  |---|---|
  | `max_vel` | 0.6 m/s |
  | `max_acc` | 1.0 m/s² |
  | Obstacle inflation | 0.3 m |
  | Waypoint tolerance | 0.3 m |
  | Waypoint timeout | 60 s |

- **Mission**: the default 12-waypoint lawnmower from `exploration_manager`.
- **Terrain filter** (`star_nav_bringup/scripts/terrain_filter.py`): the
  drone's map treats everything above the floor as an obstacle, including ramps
  and steps. The filter keeps a 2.5D grid of 0.1 m cells, each with its lowest
  lidar return (the ground) and its highest return within 0.55 m above that.

  | Cell | Result |
  |---|---|
  | Highest return more than 9 cm above the ground (wall, leg, box) | Obstacle |
  | Ground 9 cm to 0.55 m above a neighbour's (box top, platform edge) | Obstacle |
  | Ramp, step of 9 cm or less | Free |
  | Only returns above the robot (table top) | Free |

  Obstacle cells within 7 m are published as vertical columns. grid_map takes
  them in replace mode (`enable_pointcloud_raycasting` false). The robot's
  lidar looks −30° … 52° so that it sees the ground near the robot, roughly a
  forward-tilted Mid-360.

### Frontier exploration (`explore:=frontier`)

Without preset waypoints: `star_nav_bringup/scripts/frontier_explorer.py`
takes the place of `exploration_manager` and sends the planner its goals.
It needs the CMU planner (`planner:=cmu`, the default).

- **Map** (`frontier_map.py`, no ROS): a 2D grid of 0.1 m cells over the
  whole run, built from the lidar scans in the world frame. Nothing is
  forgotten. It is published on `/frontier_explorer/map`
  (`nav_msgs/OccupancyGrid`) and is separate from the planner's own map.

  | Per scan | What is kept |
  |---|---|
  | Every return | The cell's lowest return (the ground) and its highest within `robot_height` above that |
  | Rays that end no higher than the lidar, up to 8 m, at most 4,000 a scan | The cells under the ray are *seen* |
  | The robot | The cells within 0.5 m of it are seen (the lidar does not look under the robot) |

  | Cell | State |
  |---|---|
  | Obstacle by the terrain filter's rule (the table above) | Obstacle |
  | Seen, not an obstacle | Free |
  | Unknown, with three of its four neighbours known (a gap between two rays) | Free |
  | Anything else | Unknown |

- **Frontiers**: free cells the robot can reach (0.4 m clear of obstacles),
  next to an unknown cell, inside the exploration area plus `frontier_margin`.
  They are grouped into clusters of 5 cells or more.
- **Mission**: walk to the nearest cluster (by path length); when no frontier
  is left near the goal, take the next one. A frontier that takes more than
  60 s, or at which the robot stands for 6 s, is given up. When none is left
  the robot walks home. The states are on `/exploration/state`
  (`PLAN_TO_FRONTIER`, `EXEC_FRONTIER`, `RETURN_HOME`, `FINISH`).
- **Goals**: the frontier itself; the CMU planner's waypoint router finds
  the way to it. The EGO planner is not supported: it needs goals it can
  reach in a straight line.
- **Done** means no reachable frontier is left. Coverage is not part of the
  rule; `frontier_smoke.test` measures it afterwards as the share of known
  cells (free or obstacle) of the floor inside the house's walls
  (10.8 m x 10.8 m), and expects 95 %.

## Measured

Flat ground, policy command → speed reached:

| Command | Reached |
|---|---|
| forward 0.5 m/s | 0.49 m/s |
| sideways 0.3 m/s | 0.24 m/s |
| turn 0.5 rad/s | 0.45 rad/s |

Terrain (house corridor, both directions):

| Obstacle | Result |
|---|---|
| 12° ramp | Climbs |
| 7 cm steps | Reliable |
| 8 cm steps | Sometimes stalls |
| 10 cm steps | Falls |

Verification at the last commit:

| Check | Result |
|---|---|
| `lite3_smoke.test` (house, lidar; the default) | 12/12, 0 contacts, 0 timeouts, reached the platform (peak torso z 0.54 m) |
| `lite3_smoke.test world:=forest` | 12/12, 0 contacts, 0 timeouts, all 107 pillars mapped |
| `lite3_smoke.test world:=forest sensor:=depth` | 12/12, 0 contacts |
| `frontier_smoke.test` (house, CMU planner; six runs) | 15 to 20 frontiers, 98.6 to 98.8 % of the floor known, 72 to 86 s, 33 to 37 m walked, home within 0.26 m, no frontier given up, 0 contacts |
| `pytest` in `mujoco_sim/` | 30 passed; the walking tests need the assets |

## Changes Outside the Robot

The quadruped work also fixed two things that affect the drone:

- **EGO waypoint callback** (`ego_replan_fsm.cpp`): it busy-waited for
  `EXEC_TRAJ`. If the trajectory ended during that wait, the planner hung and
  ignored all further waypoints. This was behind intermittent drone lidar
  failures (2 of 4 runs). It now triggers from `WAIT_TARGET` or replans from
  `EXEC_TRAJ` directly; 4/4 runs pass.
- **Smoke test**: rostest has no `rosout` node, so `/rosout_agg` was empty and
  waypoint timeouts were never counted. The test now subscribes to `/rosout`.

## Limitations

- **Use the lidar.** It is the default and the only sensor that works in
  every world. The depth camera (`sensor:=depth`) bypasses the terrain
  filter, so it is for flat ground (the forest) only: in the house the ramp
  and the steps become obstacles, and one run fell once, timed out on 5
  waypoints and logged about 1,200 contact steps. `terrain:=false` (lidar
  without the terrain filter) is flat-ground only for the same reason: in the
  house one run reached the waypoints with about 16,000 contact steps. Both
  fail the same way before and after the robot interface refactoring.
- **The planner does not know the gait's limits.** The filter's 9 cm threshold
  is above what the policy climbs reliably (7 cm). A step between 8 and 9 cm
  would be planned over and may stall the robot.
- **Static worlds only.** Terrain cells are never cleared, so a moved object
  stays an obstacle and a removed one never frees its cells.
- **Fixed-height planning.** EGO is a 3D trajectory optimiser for a drone, used
  here in one horizontal layer. It does not plan up or down terrain, and
  heading is only what the trajectory's yaw gives.
- **Ground-truth pose.** No odometry or SLAM is in the loop.
- **Frontier exploration needs the CMU planner**, and has only been run
  with the Lite3 in the house. An earlier version for the EGO planner (goals
  on the way, in a straight free line from the robot) was dropped: in 3 of 5
  runs the robot stopped on the way, and the cause was not found.
- **The frontier map is not checked or saved.** Coverage counts known cells;
  the map is not compared with the simulator's world, and it is not written
  to a file.
- **Blind policy.** It cannot see terrain, so higher steps need a
  terrain-aware policy.
- **Tested locally only.** The model and policy are downloaded;
  `scripts/run_local_tests.sh` runs `lite3_smoke.test` when they are there.

## Next Steps

1. **Odometry**: replace the ground-truth pose with lidar-inertial odometry
   (e.g. FAST-LIO) on simulated IMU and lidar.
2. **Gait limits**: tie the filter's `max_step` to the gait's measured limit.
3. **Dynamic worlds**: clear terrain cells as rays pass through them.
4. **Terrain-aware gait**: a policy that sees the terrain (height scan) for
   higher steps.
5. **Hardware**: a Lite3 hardware branch (`hardware/*`) once the above
   are done.
