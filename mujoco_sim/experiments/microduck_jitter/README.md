# Why the Microduck shakes (2026-11)

Measurements on the simulated Microduck (`--robot microduck`, velstand v7,
`j4i6yoq2`@1250) to explain the shaky walk in the exploration demo. Run with
the simulator's Python from `mujoco_sim/`, e.g.
`.venv/bin/python experiments/microduck_jitter/mission.py current pursuit`.

| script | measures |
|---|---|
| `floor.py` | camera shake, yaw rate and speed for constant commands |
| `floor2.py` | the same with the training-faithful servo model (`faithful.py`) |
| `jitter.py` | trunk rates, height and action rate for constant commands |
| `mission.py` | replays the recorded mission (`site/assets/mujoco/microduck_setpoints.json`, open loop) through a tracker from `trackers.py` |

Camera shake is the RMS angular rate of the head depth camera, sampled at
50 Hz: what the depth images (and the map) see.

## 1. The tracker chatters between standing and walking (main cause)

| tracker | mode switches / min | stand / turn / walk | camera RMS / p95 (deg/s) | setpoint error mean / max (m) |
|---|---|---|---|---|
| `microduck.Tracker` (current) | **203** | 27 / 7 / 66 % | 70 / 118 | 0.03 / 0.34 |
| hysteresis + dwell (`Smooth`) | 32 | 5 / 29 / 66 % | 78 / 130 | 0.06 / 0.39 |
| walk at one speed, no in-place turns (`Pursuit`) | **7** | 21 / 0 / 79 % | **58 / 95** | 0.29 / 1.08 |
| `Pursuit`, turn up to 1.0 | 7 | 22 / 0 / 78 % | 58 / 96 | 0.22 / 0.71 |

The planner's setpoint often moves slower than the duck can walk (it does not
step below ~0.12 m/s), so the current tracker walks, gets ahead, waits, walks
again: 3.4 gait starts and stops per second, each one a jolt. All runs: no
contact with obstacles, no fall.

## 2. The gait itself shakes the head camera

Constant commands, flat floor, after 2 s:

| command (vx, vy, wz) | camera RMS (deg/s) | yaw rate (rad/s) | speed (m/s) |
|---|---|---|---|
| stand (0, 0, 0) | 0.3 | 0 | 0 |
| (0.3, 0, 0) | 63 | −0.03 | 0.15 |
| (0.4, 0, 0) | 104 | −0.13 | 0.21 |
| (0.3, 0, 0.5) | 67 | 0.30 | — |
| (0.3, 0, 1.0) | 88 | 0.07 | — |
| (0, 0, 1.0) | 0.2 | 0.00 | 0 |
| (0, 0, 1.5) | 72 | 0.01 | 0.10 |
| (0, 0.2, 0) | 0.5 | 0 | 0 |

- Walking at 0.3 is the floor (~60 deg/s); 0.4 and above shake much more.
- **The policy does not turn in place** in this simulation: 0.5–1.0 rad/s
  leaves it standing, 1.5 (outside the trained ±1.0) makes it step on the
  spot without turning. The current tracker's turn-in-place mode is therefore
  shaking in place. Turning works only while walking, best at about 0.5.
- Sideways commands are ignored.

## 3. Not the cause: the servo model

Upstream trains with mjlab's `BamActuator` (friction rows matched by DOF,
not by joint index as `bam.mujoco.MujocoController` does), the servo's PD and
the joint observations reading through the backlash hinge, and a 3–6 step
command delay. `faithful.py` reproduces all of that; the response barely
changes (vx 0.3: 0.16 m/s; in place 1.0: no turn; with the delay, 0.2 + 0.6
turns the wrong way). Upstream's own deployment script
(`scripts/infer_policy.py`) runs the policy exactly as we do.

Open question: upstream's bench reports in-place turning at 0.45 rad/s for
this policy; our MuJoCo (CPU) runs do not reproduce it. Candidates not yet
checked: MuJoCo Warp vs CPU contact differences, the rough-terrain training
distribution, solver settings (`iterations` 30 / `ls_iterations` 50 in
training).

## What to change

1. Replace `Tracker` with a pursuit tracker: one calm forward command (0.3),
   steer while walking, no in-place turns, stand only at resting setpoints,
   with hysteresis and a minimum dwell.
2. Let the planner move at the duck's walking speed (~0.15 m/s) so the duck
   rarely gets ahead of the setpoint.
3. Verify closed loop (`exploration_microduck.launch`): the replay above is
   open loop, so its tracking error overstates what the replanning planner
   sees.
