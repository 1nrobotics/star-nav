# Quadruped (Lite3): Development Status

Status of the walking-robot path on the `feature/quadruped` branch: what
works, how it fits together, what was measured, and what is missing. How to
run it: [SIMULATION.md](SIMULATION.md#mujoco-simulator) and
[mujoco_sim/README.md](../mujoco_sim/README.md#lite3-quadruped).

## Summary

| Area | Status |
|---|---|
| Lite3 in MuJoCo, upstream walking policy | Working |
| CMU local planner driving the robot | Working: terrain analysis, local planner, path follower; velocity commands |
| House with the lidar: furniture, 12° ramp, 0.21 m platform, 7 cm steps | 12/12 waypoints, no contacts |
| Frontier exploration (`explore:=frontier`) | Working in the house |
| Depth camera | Not supported: the planner takes the lidar's scans |
| Localisation | Simulator ground truth |
| Steps above 8 cm | Not supported (blind policy) |
| Hardware | None |
| Tests | Local: `scripts/run_local_tests.sh lite3` (needs the downloaded assets) |

## Architecture

The Lite3 is a legged robot, so it walks with the CMU local planner
([PLANNER_INTERFACE.md](PLANNER_INTERFACE.md)), on the lidar's scans:

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

Until November 2026 the quadrotor's EGO planner could move the robot too
(`planner:=ego`: position setpoints at a fixed height, on a terrain filter's
map). That path was removed; what it did and how it compared is in
[GROUND_PLANNING.md](GROUND_PLANNING.md).

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
- **Command**: the planner's velocity (forward, turn rate) goes to the
  policy within 0.8 m/s and 0.8 rad/s (`tracking.Holonomic.limit`). The
  simulator can also steer the robot to position setpoints
  (`tracking.Holonomic.twist`, which the EGO path used); no planner sends
  them now.
- **Safety in the simulator**: a fall is logged when the torso drops below half
  the standing height. Contacts with the ground count as walking, not
  collisions; so do feet and lower legs on `walk/` geoms (ramp, platform,
  steps).

### Planning

- **Planner**: the CMU local planner; see
  [GROUND_PLANNING.md](GROUND_PLANNING.md#10-the-cmu-planner-in-star-nav)
  for how it is wired and its parameters. The robot's sizes are in
  `star_nav_bringup/launch/exploration_lite3.launch`, the rest in
  `exploration_ground.launch`:

  | Parameter | Value |
  |---|---|
  | `max_vel` | 0.6 m/s |
  | `robot_height` (higher returns are overhead) | 0.55 m |
  | Obstacle height (`cmu_obstacle_height`) | 0.15 m above the ground |
  | Waypoint tolerance | 0.3 m |
  | Waypoint timeout | 60 s |

- **Mission**: the default 12-waypoint lawnmower from `exploration_manager`.
- **Lidar**: it looks −30° … 52° so that it sees the ground near the robot,
  roughly a forward-tilted Mid-360.

### Frontier exploration (`explore:=frontier`)

Without preset waypoints: `star_nav_bringup/scripts/frontier_explorer.py`
takes the place of `exploration_manager` and sends the planner its goals.

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
  | Highest return more than 9 cm (`max_step`) above the ground (wall, leg, box) | Obstacle |
  | Ground 9 cm to 0.55 m above a neighbour's (box top, platform edge) | Obstacle |
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
  the way to it.
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

Verification on 2026-11-26, with the CMU planner as the only one (one run
each; times are simulated):

| Check | Result |
|---|---|
| `lite3_smoke.test` (house; the default) | 12/12, 0 contacts, 0 timeouts, reached the platform (peak torso z 0.54 m), 143 s |
| `lite3_smoke.test world:=forest` | 12/12, 0 contacts, 0 timeouts, 136 s |
| `frontier_smoke.test` (house) | 18 frontiers, 98.6 % of the floor known, 80 s, 36.2 m walked, home within 0.25 m, 0 contacts (six earlier runs: 15 to 20 frontiers, 98.6 to 98.8 %, 72 to 86 s) |
| `pytest` in `mujoco_sim/` | 63 passed (with both robots downloaded) |

The forest test no longer compares a map with the pillars: that check read
EGO's map.

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

- **Lidar only.** The CMU planner's terrain analysis takes lidar scans;
  there is no depth-camera mode for the walking robots.
- **The planner does not know the gait's limits.** A point 15 cm above the
  ground is an obstacle (`cmu_obstacle_height`); the policy climbs 7 cm
  steps reliably. A step between 8 and 15 cm is not an obstacle to the
  planner and may stall the robot.
- **Static worlds only.** The terrain analysis is set up for a world that
  does not change (`clearDyObs` false).
- **Ground-truth pose.** No odometry or SLAM is in the loop.
- **Frontier exploration has only been run with the Lite3 in the house.**
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
2. **Gait limits**: tie the planner's obstacle height to the gait's
   measured limit.
3. **Dynamic worlds**: let the terrain analysis clear what has moved.
4. **Terrain-aware gait**: a policy that sees the terrain (height scan) for
   higher steps.
5. **Hardware**: a Lite3 hardware branch (`hardware/*`) once the above
   are done.
