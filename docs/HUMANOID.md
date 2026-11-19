# Humanoid (Booster T1): Development Status

The T1 runs the same exploration mission and planner as the quadrotor and the
Lite3 quadruped, in the MuJoCo simulator only. This page records how it is
built, what was measured and what is still missing. The quadruped's page,
[QUADRUPED.md](QUADRUPED.md), covers the parts the two walking robots share
(terrain filter, tracker, planner settings) in more detail.

## Summary

- **Robot**: Booster Robotics' T1 (1.16 m, 12 actuated leg joints, arms
  fixed), the model of their training framework
  [booster_gym](https://github.com/BoosterRobotics/booster_gym).
- **Walking**: booster_gym's trained velocity policy, run in NumPy (no
  PyTorch). Model and policy (Apache-2.0) are downloaded at a pinned commit by
  `./scripts/setup_mujoco_env.sh --t1`, not committed.
- **Planning**: unchanged. EGO plans at trunk height (0.67 m); the lidar on the
  head feeds the terrain filter, which lets it walk the house's ramp and steps.
- **Launch / test**: `exploration_t1.launch`, `t1_smoke.test` (house, lidar
  by default; `world:=forest`).

## Architecture

```
waypoints ─► EGO-Planner (0.67 m, 0.5 m/s, 0.35 m inflation) ─► pos_cmd
                ▲                                                   │
   terrain filter (2.5D, returns up to 1.25 m above the ground)     ▼
                ▲                                  Tracker (setpoint → vx, vy, ωz)
   lidar on the head (1.2 m, −50° … 30°)                            │
                                                   policy, 50 Hz (NumPy MLP)
                                                                    │
                                                   joint PD, 500 Hz ─► MuJoCo
```

### Control (`mujoco_sim/star_nav_mujoco/t1.py`)

- **Model**: `T1_locomotion.xml` with its 24 meshes, attached to the scene with
  `MjSpec` under the `t1/` prefix. A depth camera goes in the face and a lidar
  housing on the head. The meshes move to geom group 2 and the collision geoms
  to group 3, so neither sensor sees the robot.
- **Policy**: `T1.pt` is a TorchScript actor (47 → 256 → 128 → 128 → 12, ELU).
  `extract_weights` reads its eight tensors straight from the archive's raw
  `data/<i>` files, checks their sizes, and saves `policy.npz`. Nothing is
  unpickled, so no code from the download runs.
- **Observation** (as `play_mujoco.py`): gravity in the body frame, body
  angular velocity, the velocity command, a gait clock (cos and sin of a
  1.5 Hz phase, zero and stopped on a zero command so the robot stands), joint
  positions minus the standing pose, 0.1 × joint velocities, and the last
  action.
- **Action**: joint targets = standing pose + action (clipped to ±1); PD with
  kp 200 and kd 5 on hips and knees, 50 and 1 on ankles, at every 2 ms step,
  clipped to the model's torque limits.
- **Steering**: the Lite3's `Tracker` (setpoint velocity plus 1.5/s ×
  position error, turn to the planner's yaw, stand on a resting setpoint).

### Planning

Everything is in `exploration_legged.launch` (`robot:=t1`), shared with the
Lite3. Per robot, it sets the walking height, the lidar mount and beam angles,
the terrain filter's overhead height, the inflation, the planner speed, and
the top of the map shown in RViz.

| | Lite3 | T1 |
|---|---|---|
| Planner height | 0.34 m | 0.67 m |
| Lidar | back, 0.44 m, −30° … 52° | head, 1.2 m, −50° … 30° |
| Overhead above | 0.55 m | 1.25 m (the table is an obstacle) |
| Inflation | 0.3 m | 0.35 m (arms reach 0.27 m) |
| Speed | 0.6 m/s | 0.5 m/s |
| House walls | 1.0 m | 1.6 m |

## Measured

Flat ground, policy command → speed reached:

| Command | Reached |
|---|---|
| forward 0.5 m/s | 0.42 m/s |
| sideways 0.3 m/s | 0.26 m/s |
| turn 0.5 rad/s | 0.51 rad/s |

Terrain (house corridor, setpoint moving at 0.2–0.6 m/s):

| Obstacle | Result |
|---|---|
| 12° ramp up, 7 cm steps down | 9 of 10 runs (one fall walking 0.3 rad off the corridor's axis) |
| 7 cm steps up | Falls in 11 of 12 runs |

Verification at the last commit:

| Check | Result |
|---|---|
| `t1_smoke.test` (house, lidar; the default) | 12/12, 0 contacts, 0 timeouts, reached the platform (peak trunk z 0.88 m), 236 s |
| `t1_smoke.test world:=forest` | 12/12, 0 contacts, 0 timeouts, all 84 pillars mapped |
| `t1_smoke.test world:=forest sensor:=depth` | 12/12, but 36 contact steps (see Limitations) |
| `lite3_smoke.test`, `mujoco_smoke.test`, `sim_smoke.test` | pass (regression) |
| `pytest` in `mujoco_sim/` | 39 passed (with both robots downloaded) |

## Limitations

- **Cannot climb the house's steps.** The policy was trained on terrain with
  steps of about 2 cm. Going down 7 cm steps works; going up does not. The
  mission goes up the ramp and down the steps, and the terrain filter (9 cm)
  does not stop the planner from routing up the steps.
- **Depth camera: lidar only for now.** Without the terrain filter, EGO plans
  in the 3D map and may go over an obstacle lower than about 1 m (the
  house's table, a short pillar); the T1 cannot follow, and brushes it
  (`t1_smoke.test world:=forest sensor:=depth`: 12/12 waypoints, but 36
  contact steps with a stump). A virtual ceiling just above the walking
  height fixes this; it is implemented on the `feature/exploration` branch.
- **Arms have no collision geoms** in the upstream model. The 0.35 m
  inflation keeps them clear, but a touch with an arm would not be counted.
- Shared with the Lite3: static worlds only, fixed-height planning,
  ground-truth pose, a blind policy, and no CI (the assets are downloaded).

## Next Steps

1. **Virtual ceiling** for walking robots (from `feature/exploration`), so
   the depth camera works too.
2. **Gait limits in the planner**: a per-direction step limit, or a terrain
   cost, so it never plans up steps the policy cannot climb.
3. **A terrain-aware policy** (height scan) for real stairs.
4. **Odometry** in place of the ground-truth pose, as for the Lite3.
5. **Hardware**: a T1 hardware branch (`hardware/*`) once the above are done.
