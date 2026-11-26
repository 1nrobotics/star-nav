# Humanoid (Booster T1): Development Status

The T1 runs the same exploration mission as the quadrotor and the Lite3
quadruped, with the Lite3's planner (CMU's local planner), in the MuJoCo
simulator only. This page records how it is
built, what was measured and what is still missing. The quadruped's page,
[QUADRUPED.md](QUADRUPED.md), covers the parts the two walking robots share
(the CMU planner's settings, the velocity command) in more detail.

## Summary

- **Robot**: Booster Robotics' T1 (1.16 m, 12 actuated leg joints, arms
  fixed), the model of their training framework
  [booster_gym](https://github.com/BoosterRobotics/booster_gym).
- **Walking**: booster_gym's trained velocity policy, run in NumPy (no
  PyTorch). Model and policy (Apache-2.0) are downloaded at a pinned commit by
  `./scripts/setup_mujoco_env.sh --t1`, not committed.
- **Planning**: CMU's local planner, as for the Lite3. It sends the policy
  a velocity; the lidar on the head feeds its terrain analysis, to which the
  house's ramp and steps are ground. It is the only planner of a legged
  robot ([PLANNER_INTERFACE.md](PLANNER_INTERFACE.md)); `planner:=ego`
  stops the launch. Until November 2026 the T1 could also follow EGO at
  trunk height behind a terrain filter; results on this page that name EGO
  or the depth camera are from then.
- **Launch / test**: `exploration_t1.launch`, `t1_smoke.test` (house, lidar
  by default; `world:=forest`).

## Architecture

The chain is the Lite3's ([QUADRUPED.md](QUADRUPED.md#architecture),
[GROUND_PLANNING.md](GROUND_PLANNING.md#10-the-cmu-planner-in-star-nav)):

```
waypoints ─► waypoint router (A* on the terrain map) ─► local planner ─► path follower
                ▲                                        (0.5 m/s)           │
   terrain analysis (returns up to 1.25 m above the ground)          velocity (vx, ωz)
                ▲                                                            ▼
   lidar on the head (1.2 m, −50° … 30°)                    policy, 50 Hz (NumPy MLP)
                                                                             │
                                                            joint PD, 500 Hz ─► MuJoCo
```

### Control (`mujoco_sim/star_nav_mujoco/robots/t1.py`)

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
- **Steering**: the planner's velocity (forward speed and turn rate) is the
  policy's command. The simulator can also steer the T1 to position
  setpoints with the Lite3's tracker, `tracking.Holonomic` (setpoint
  velocity plus 1.5/s × position error, turn to the planner's yaw, stand on
  a resting setpoint); that was the EGO route, and no launch file uses it
  now.

### Planning

`exploration_t1.launch` sets the robot's type (`legged`) and sizes: the
walking height, the lidar mount, the height above which returns are
overhead, the clearance and the planner speed. Everything else is in
`exploration_ground.launch`, shared with the Lite3. The lidar's beam angles
are in `robots/t1.py`.

| | Lite3 | T1 |
|---|---|---|
| Planner | CMU | CMU |
| Body height (waypoints) | 0.34 m | 0.67 m |
| Lidar | back, 0.44 m, −30° … 52° | head, 1.2 m, −50° … 30° |
| Overhead above | 0.55 m | 1.25 m (the table is an obstacle) |
| Clearance (frontier goals) | 0.3 m | 0.35 m (arms reach 0.27 m) |
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

Verification on 2026-11-26, with the CMU planner (one run each; times are
simulated):

| Check | Result |
|---|---|
| `t1_smoke.test` (house; the default) | 12/12, 0 contacts, 0 timeouts, reached the platform (peak trunk z 0.87 m), 183 s |
| `t1_smoke.test world:=forest` | 12/12, 0 contacts, 0 timeouts, 166 s |

When the T1 was added it followed EGO (`planner:=ego`, since removed; these
rows cannot be repeated from the launch files):

| Check | Result |
|---|---|
| `t1_smoke.test` (house, lidar; the default) | 12/12, 0 contacts, 0 timeouts, reached the platform (peak trunk z 0.88 m), 236 s |
| `t1_smoke.test world:=forest` | 12/12, 0 contacts, 0 timeouts, all 84 pillars mapped |
| `t1_smoke.test world:=forest sensor:=depth` | 12/12, but 36 contact steps (see Limitations) |
| `lite3_smoke.test`, `mujoco_smoke.test`, `sim_smoke.test` | pass (regression) |
| `pytest` in `mujoco_sim/` | 39 passed (with both robots downloaded) |

## Limitations

- **Cannot climb the house's steps.** The policy was trained on terrain with
  steps of about 2 cm. Going down 7 cm steps works in most runs (see
  [Measured](#measured)); going up does not. The mission goes up the
  ramp and down the steps, but nothing stops the planner from routing up
  the steps: to its terrain analysis anything lower than 0.15 m is ground.
- **Lidar only.** The CMU planner's terrain analysis takes lidar scans;
  there is no depth-camera mode. (With EGO and the depth camera the T1
  brushed a stump in the forest: EGO planned over an obstacle lower than
  the robot.)
- **Arms have no collision geoms** in the upstream model, so a touch with
  an arm would not be counted. The CMU planner's paths keep 0.45 m clear
  (0.23 m where nothing else is free); the arms reach 0.27 m.
- **A fall ends the mission.** There is no get-up controller (see
  [Falls and getting up](#falls-and-getting-up)), and a fall is only logged:
  the smoke test fails later, on a waypoint timeout.
- Shared with the Lite3: static worlds only, ground-truth pose, a blind policy, and tests that run locally only (the
  assets are downloaded).

## Falls and getting up

**Today.** The simulator logs a fall when the trunk drops below half its
standing height (`sim.py`, `_step_legged`). The walking policy keeps running,
but it was trained for walking only and does not get up. A
robot lying on the ground plane counts no contacts, so the smoke test fails
only when the next waypoint times out.

**Why a get-up policy cannot simply be added.** `T1_locomotion.xml` actuates
the 12 leg joints only; the arms are fixed. Getting up needs the arms, so it
needs the full-body T1 model first, then a policy trained for it.

### Published work

Read in November 2026 from the papers' arXiv pages and repositories. The
numbers are each paper's own; trial counts and test conditions differ, so
they do not rank the methods.

| Work | Robot | Method | Reported results | Code |
|---|---|---|---|---|
| **HiFAR** (IROS 2025, [2502.20061](https://arxiv.org/abs/2502.20061)); Tsinghua and Booster Robotics | **Booster T1**, 23 DoF | PPO on booster_gym in two stages: lying on the back or front in the sagittal plane with loose limits, then 3D cases (crossed legs) with real joint, torque and speed limits. Episodes start from six key poses of a hand-made get-up motion. | Real robot: 20/20 from the back, 20/20 from the front, about 2.8 s. Simulation (Webots): 10 trials per condition. | [Hi-FAR/HiFAR](https://github.com/Hi-FAR/HiFAR): training code, T1 resources and a MuJoCo play script. No weights listed. |
| **FRASA** (preprint, 2024, [2410.08655](https://arxiv.org/abs/2410.08655)); Rhoban, Bordeaux | Sigmaban, a servo-driven RoboCup KidSize humanoid, 20 DoF | CrossQ, trained in MuJoCo. Five joints in the sagittal plane (elbow, shoulder, hip, knee and ankle pitch), mirrored left and right. From the back and the front. | Validated on the real robot; reported better than the team's hand-tuned keyframes. No numbers on the pages read. | [Rhoban/frasa](https://github.com/Rhoban/frasa), MIT, with the Sigmaban MuJoCo model. No weights listed. |
| **HoST** (RSS 2025, best systems paper finalist, [2502.08378](https://arxiv.org/abs/2502.08378)) | Unitree G1, 23 DoF; also H1, Mini Pi | PPO from scratch, no motion data, proprioception only. Four reward groups with a critic each; an upward pull that starts at 200 N and is reduced as it succeeds; an action bound that tightens; a smoothness regulariser (L2C2). | Simulation: 99.5 % ground, 99.8 % platform, 94.2 % wall, 98.5 % slope. Real: 20/20 over the four terrains. Without the pull curriculum or the separate critics: 0 %. | [OpenRobotLab/HoST](https://github.com/OpenRobotLab/HoST), MIT, Isaac Gym. No weights listed. |
| **HumanUP** (RSS 2025, [2502.12152](https://arxiv.org/abs/2502.12152)) | Unitree G1, 23 DoF controlled | Two stages. I: find a get-up (or roll-over) motion with a simplified model and weak limits. II: track that motion slowed 8×, with the full model, strong smoothness and torque limits, and randomised postures and terrain. No human motion data. | Simulation: 95.3 % from the back, 92.1 % from the front. Real, six outdoor surfaces: 78.3 % from the back, against 41.7 % for the manufacturer's controller; about 6 s. | [RunpeiDong/humanup](https://github.com/RunpeiDong/humanup), Isaac Gym |
| **FIRM** (ICRA 2026, [2511.07407](https://arxiv.org/abs/2511.07407)) | Unitree G1, 23 DoF | One policy for avoiding a fall, falling with less impact, and getting up. RL tracks 25 key frames from each of five human videos; the result is distilled into a diffusion policy. | Simulation: 96.3 % flat, 93.2 % uneven, 55.9 % wavy ground. Real: 10/10 on a mat, 8/10 on a slippery floor. | Not stated |
| **ReLoco** (preprint, November 2026, [2611.23916](https://arxiv.org/abs/2611.23916)) | Tiangong 3.0, 64.5 kg, 23 DoF controlled | One proprioceptive policy for walking, getting up and resuming the command. Adversarial motion priors with one discriminator and an output per behaviour; references are retargeted human motions replayed by a tracking policy in simulation; an upward pull from 400 N, reduced to zero. | Simulation: get-up 96.8 %, push recovery 96.96 %. Real: 30/30 get-ups in 2.64 ± 0.64 s; recovered from all 34 falls where control continued (47 kicks, 8 stopped by hardware protection). | Not stated |
| **ROOF** (preprint, October 2026, [2610.42097](https://arxiv.org/abs/2610.42097)) | Unitree G1, 29 DoF | Getting up under an overhead obstacle (a table). A reward for reducing the overlap between robot and obstacle, plus a diffusion motion prior from LAFAN1 get-up clips. The policy sees proprioception only. | Simulation: 88.5 % under plates it must lift (18.9 % without the guidance). Real: 95 of 150 over seven scenes. | Not stated |

Stubborn ([2606.12814](https://arxiv.org/abs/2606.12814), preprint, G1) is
about staying on, or returning to, a tracked motion after a 5 m/s push; it
does not describe getting up from the ground.

What the papers agree on:

- **Proprioception only.** None of the policies sees the terrain.
- **An assisting upward force during training** (HoST, ReLoco) or starting
  from key poses of a known motion (HiFAR). HoST without it reaches 0 %.
- **Smoothness has to be forced.** HoST without its smoothness term drops
  from 20/20 to 11/20 on the real robot; HumanUP's second stage exists for
  the same reason.
- **Arms are used.** FRASA drives shoulders and elbows; the others control
  23 or more joints.
- **Both lying postures in one policy are harder.** HoST reports that
  training on back and front together hurt; HumanUP rolls over first.

Where they disagree: FIRM and ReLoco both compare against HoST with their
own re-implementations. FIRM reports HoST at 23 % on uneven ground and 1/10
on the real robot, and says its HoST numbers differ from the HoST paper's.
ReLoco reports HoST slightly ahead on random fallen states (99.9 % against
98.3 %, and 1.57 s against 2.02 s). Treat cross-paper comparisons as weak.

### What HiFAR adds

HiFAR positions itself against FRASA, HoST and HumanUP:

1. **Two stages, 2D then 3D.** Stage 1 controls only the joints that move
   in the sagittal plane, from the back and the front, with loose limits
   (wider joint range, more torque). Stage 2 adds hip roll for 3D cases
   such as crossed legs, with the real position, torque and speed limits,
   pushes and terrain randomisation. FRASA stays in the plane throughout.
2. **The network grows instead of restarting.** Stage 2 adds
   zero-initialised layers at the input and output of the stage-1 network,
   so it starts out behaving exactly as stage 1 did.
3. **A mirror loss instead of forced symmetry.** Symmetry is encouraged,
   not built in as in FRASA, so sideways and crossed-leg falls are possible.
4. **Starting from key poses.** Each episode starts from one of six poses
   (three prone, three supine) of a hand-made get-up motion. This does the
   job of HoST's assisting pull, without an external force.

Its stated objections: FRASA is planar and symmetric; HoST's sub-tasks may
not suit prone and sideways falls; HumanUP tracks a motion slowed 8×, which
costs speed (HiFAR about 2.8 s on the T1, HumanUP about 6 s on the G1, not
measured against each other).

Weak points of the paper:

- No ablations, so the effect of each of the four parts is not shown.
- No quantitative comparison with any other method.
- It needs a hand-made get-up motion for the six poses; HoST needs none.
- Simulation tests use 10 trials per condition; on the real robot only the
  two lying postures have counts.

### For the T1

- **HiFAR is the direct match**: the same robot and the same training
  framework (booster_gym) as our walking policy, with a MuJoCo play script.
  It needs the 23-DoF T1 model, and a policy has to be trained (Isaac Gym,
  GPU); its licence and the model files were not checked.
- **FRASA is the lighter route**: it trains in MuJoCo, without Isaac Gym,
  but covers planar, symmetric get-ups only and would need a T1
  environment written for it.
- **The real T1 has a built-in get-up.** Booster's SDK
  ([booster_robotics_sdk](https://github.com/BoosterRobotics/booster_robotics_sdk),
  `b1_loco_api.hpp`) has `kGetUp` and `kGetUpWithMode` for the T1. That is a
  firmware action; it does not run in our simulator.

### Options, cheapest first

1. **Report the fall**: a flag in the state message (`protocol.py`,
   `protocol.h`, version bump) and an assertion in the smoke test.
2. **Reset in simulation**: put the robot back on its feet and continue.
   Keeps long missions running; it is not getting up.
3. **Train a get-up policy with HiFAR** on the 23-DoF T1 model, and switch
   between it and the walking policy on the fall flag. A separate training
   project; on hardware the SDK's `kGetUp` does this already.

Keeping the planner off steps the T1 cannot take (Next Steps 1) removes the
main cause of falls and comes before any of these.

## Next Steps

1. **Gait limits in the planner**: a per-direction step limit, or a terrain
   cost, so it never plans up steps the policy cannot climb.
2. **A terrain-aware policy** (height scan) for real stairs.
3. **Odometry** in place of the ground-truth pose, as for the Lite3.
4. **Falls**: report a fall in the state message and fail the smoke test
   on it (see [Falls and getting up](#falls-and-getting-up)).
5. **Hardware**: a T1 hardware branch (`hardware/*`) once the above are done.
