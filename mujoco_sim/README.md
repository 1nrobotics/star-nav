# star_nav_mujoco

Standalone MuJoCo 4 quadrotor simulator for Star Nav. It replaces the
kinematic simulator (`poscmd_2_odom` + `pcl_render_node` + `random_forest`)
with rigid-body dynamics, a flight controller, a GPU/OSMesa depth camera or a
360° lidar, collision detection and a browser viewer. It does not use ROS.

```
 mujoco_sim (Python >= 3.11, no ROS)            ROS Noetic
 ┌────────────────────────────────────┐         ┌──────────────────────────┐
 │ forest world + quadrotor (MJCF)    │  state  │ mujoco_bridge (C++)      │
 │ physics 500 Hz, SE(3) controller   │ ──────► │  odom, depth, camera_pose│──► EGO planner
 │ depth camera 30 Hz (640x480)       │  depth  │  global_cloud, contacts  │    exploration
 │ collision counter                  │  world  │                          │
 │ web viewer  http://localhost:8080  │ ◄────── │  pos_cmd                 │◄── traj_server
 └────────────────────────────────────┘   cmd   └──────────────────────────┘
                 ZeroMQ tcp://127.0.0.1:5555 (sim → ROS), :5556 (ROS → sim)
```

The bridge publishes the same topics as the kinematic simulator, so the
planner, exploration manager and RViz configuration are unchanged.

ROS runs on the simulator's clock: the launch file sets `/use_sim_time` and
the bridge publishes `/clock` from the simulation time. If the machine cannot
keep up (`rt` below 1.00 in the simulator log), the whole stack slows down with
the physics instead of commanding the quadrotor faster than it can fly. Pass
`use_sim_time:=false` to `exploration_mujoco.launch` to use wall-clock time.

## Run

Docker (any OS):

```bash
./scripts/run_mujoco_smoke_test.sh        # headless end-to-end test
docker compose up -d mujoco               # simulator + viewer on :8080
```

Native (Ubuntu 20.04 with ROS Noetic, plus any Python >= 3.11):

```bash
./scripts/setup_mujoco_env.sh             # creates mujoco_sim/.venv
roslaunch star_nav_bringup exploration_mujoco.launch use_rviz:=true
rostopic pub -1 /exploration/trigger geometry_msgs/PoseStamped "{}"
```

Open http://localhost:8080 to watch the flight; the camera follows the
quadrotor. The launch file starts the simulator through
`star_nav_bringup/scripts/mujoco_sim.sh`; set `MUJOCO_SIM_PYTHON` to use
another interpreter, or `start_sim:=false` if the simulator already runs.

The simulator alone:

```bash
cd mujoco_sim && .venv/bin/python -m star_nav_mujoco --help
```

| Option | Default | Meaning |
| --- | --- | --- |
| `--robot` | quadrotor | `quadrotor`, `lite3` (quadruped) or `t1` (humanoid), see below |
| `--fetch-lite3`, `--fetch-t1` | | Download the Lite3 or T1 model and policy, then exit |
| `--world` | forest | `forest` or `house` (rooms, ramp, platform, stairs; `house.py`) |
| `--seed` | 1 | Forest layout |
| `--map-size X Y` | 50 50 | Forest extent (m) |
| `--pillars`, `--rings` | 100, 50 (lite3, t1: 0 rings) | Obstacle counts |
| `--start X Y Z` | 0 0 1 (walking robots ignore Z) | Spawn and initial hover point |
| `--bind` | 127.0.0.1 | ZeroMQ address (`0.0.0.0` for a bridge on another host) |
| `--sensor` | depth | `depth`, `lidar` or `both` ([docs/SENSORS.md](../docs/SENSORS.md)) |
| `--depth-rate` | 30 | Depth camera rate (Hz) |
| `--lidar-*` | 32 x 1024, 10 Hz | Lidar beams, columns, vertical FOV, rate, range, noise |
| `--realtime-factor` | 1.0 | Simulated seconds per wall-clock second |
| `--no-viewer`, `--viewer-port` | off, 8080 | Web viewer |
| `--record FILE` | off | Write an MP4 from the viewer's follow camera, with lidar returns as dots (needs `ffmpeg`) |

The web viewer listens on all network interfaces (a MuJoCo Studio default
that cannot be changed yet); do not expose the port on untrusted networks.

## Lite3 quadruped

DeepRobotics' Lite3 walks with DeepRobotics' own velocity policy:

```bash
./scripts/setup_mujoco_env.sh --lite3     # onnxruntime + model + policy
roslaunch star_nav_bringup exploration_lite3.launch use_rviz:=true
rostest star_nav_bringup lite3_smoke.test                  # house, lidar
rostest star_nav_bringup lite3_smoke.test world:=forest
rostest star_nav_bringup lite3_smoke.test world:=forest sensor:=depth
```

The default is the house (ramp, stairs, furniture) with the lidar. The depth
camera bypasses the terrain filter, so use it in the forest (flat ground).

- Model and policy are downloaded at pinned commits into
  `~/.cache/star_nav/lite3` (`STAR_NAV_LITE3_DIR` to change), not committed:
  MJCF and meshes from
  [deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model),
  `policy.onnx` from
  [Lite3_rl_deploy](https://github.com/DeepRoboticsLab/Lite3_rl_deploy) (both
  BSD-3-Clause; the policy is checked by SHA-256).
- Walking: the policy runs at 83 Hz on body angular velocity, gravity
  direction, the velocity command, joint positions and velocities and its
  last action, as upstream's ONNX runner; joint PD (kp 30, kd 1, 30 N m) at
  every 2 ms physics step. Measured on flat ground: forward 0.5 → 0.49 m/s,
  sideways 0.3 → 0.24 m/s, turn 0.5 → 0.45 rad/s.
- `lite3.Tracker` turns the planner's setpoint into a body-frame command
  (setpoint velocity plus 1.5/s × position error, at most 0.8 m/s) and turns
  towards the planner's yaw; it stands on a resting setpoint.
- Sensors: the depth camera at the front of the torso, the lidar 10 cm above
  the torso centre (`exploration_legged.launch`: −30° … 52°, about what a
  Mid-360 tilted forward sees ahead). Neither sees the robot's own legs
  (geom group 3).
- Terrain: on the lidar path, `star_nav_bringup/scripts/terrain_filter.py`
  keeps a 2.5D grid (lowest return per 0.1 m cell, highest within 0.55 m
  above it) and gives the planner obstacle columns: walls, legs and boxes
  higher than 9 cm are obstacles; ramps, steps and overhead table tops are
  not (`terrain:=false` feeds the lidar straight into the 3D map instead).
- What the policy can walk (measured, closed loop at 0.25–0.6 m/s): a 12°
  ramp, 7 cm steps up and down; 8 cm steps sometimes stall it, 10 cm steps
  make it fall. It is a blind flat-ground policy, so `house.py` uses 7 cm
  steps.
- Contacts with the floor, and lower legs or feet on ramps and steps
  (`walk/...` geoms), do not count; anything else does.
- The planner moves at 0.6 m/s with 0.3 m inflation, waypoints at torso
  height.

The house (`--world house`, 11 x 11 m) fits the default 9 m lawnmower
mission: a furnished room with a table to walk under, and a corridor on one
row with a ramp up to a 0.21 m platform and three steps down.

## T1 humanoid

Booster Robotics' T1 (1.2 m, 12 actuated leg joints) walks with Booster
Robotics' own velocity policy, through the same house and planner:

```bash
./scripts/setup_mujoco_env.sh --t1        # model + policy (no extra Python packages)
roslaunch star_nav_bringup exploration_t1.launch use_rviz:=true
rostest star_nav_bringup t1_smoke.test                     # house, lidar
rostest star_nav_bringup t1_smoke.test world:=forest
```

Use the lidar. The depth camera (`sensor:=depth`) bypasses the terrain
filter, and in the 3D map EGO may plan over an obstacle lower than about 1 m
(the house's table, a short pillar), which a walking robot cannot follow: in
the forest the T1 then brushes a stump with a foot. A virtual ceiling just
above the walking height would stop that.

- Model and policy are downloaded at a pinned commit of
  [booster_gym](https://github.com/BoosterRobotics/booster_gym) (Apache-2.0)
  into `~/.cache/star_nav/t1` (`STAR_NAV_T1_DIR` to change), not committed:
  `T1_locomotion.xml` with its meshes, and the trained actor `T1.pt`
  (checked by SHA-256). `T1.pt` is a TorchScript MLP; `t1.py` reads its
  weights from the archive's raw tensor files (no unpickling, no PyTorch) and
  runs it in NumPy.
- Walking: the policy runs at 50 Hz on gravity direction, body angular
  velocity, the velocity command, a 1.5 Hz gait clock (stopped on a zero
  command, so it stands), leg joint positions and velocities and its last
  action, as upstream's `play_mujoco.py`; joint PD (hips and knees kp 200,
  kd 5; ankles 50, 1) at every 2 ms physics step. Measured on flat ground:
  forward 0.5 → 0.42 m/s, sideways 0.3 → 0.26 m/s, turn 0.5 → 0.51 rad/s.
- Steering: the Lite3's `Tracker`. Sensors: the depth camera in the face
  (1.07 m), the lidar on top of the head (1.2 m, −50° … 30°); neither sees
  the robot (its meshes are in group 2, its collision geoms in group 3).
- The planner walks it at 0.5 m/s, waypoints at trunk height (0.67 m), with
  0.35 m inflation (the arms reach 0.27 m out). The terrain filter counts
  returns up to 1.25 m above the ground, so the table it cannot walk under
  is an obstacle.
- What the policy can walk (measured): the 12° ramp and the 7 cm steps
  down; climbing the 7 cm steps fails most of the time (it was trained on
  terrain with ~2 cm steps). The mission goes up the ramp and down the steps.
- In the house, its walls are 1.6 m high instead of 1 m.

## Model

- World: pillars and tilted rings drawn from the same distributions as
  `map_generator/random_forest`, deterministic per seed, with the spawn point
  kept clear.
- Quadrotor: 1 kg, X configuration, four thrust actuators with a 20 ms
  first-order motor lag, yaw from rotor drag torque, linear body drag.
- Controller: geometric SE(3) tracking (Lee et al., 2010) of the planner's
  position, velocity, acceleration and yaw, with tilt limit and anti-windup.
  If commands stop for 0.5 s the quadrotor holds the last position.
- Depth camera: 640x480, fx = fy = 387.2, principal point at the image centre,
  mounted 0.14 m ahead of the centre, looking along body x. Depth is published
  in millimetres (16UC1, 0 = no return beyond 10 m).
- Lidar (`--sensor lidar`): 32 beams over −22.5° … +22.5°, 1024 columns,
  10 Hz, 0.3–10 m, mounted 6 cm above the body centre. Each revolution is
  cast at one instant with `mj_multiRay` in parallel threads; points are sent
  in the sensor frame with the sensor pose. Sensors ignore visual-only
  markers (geom group 2), such as the setpoint sphere.
- Unlike the kinematic simulator, both sensors also see the ground, so the
  planner's occupancy map contains the floor (below 0.35 m with inflation,
  well under the 1 m flight altitude), as it would with a real sensor. RViz
  hides map voxels below 0.35 m (`visualization_min_height`) so obstacles stay
  visible.
- Textures (`textures.py`): wooden floor, plaster walls, wood, fabric,
  concrete, grass and bark, generated with NumPy when the model is built.
  They are for show: sensors measure geometry, and the house floor is a
  visual-only geom. The Lite3's meshes, one flat grey upstream, get a dark
  shell, bare motor housings and rubber feet (`lite3.APPEARANCE`).
- Contacts: the number of physics steps in which the quadrotor touched
  anything, published by the bridge on `/mujoco/contacts`. The simulator log
  names the geoms and the position of each new contact.
- The quadrotor is 0.19 m from centre to prop tip, so
  `exploration_mujoco.launch` sets the planner's obstacle inflation to 0.2 m
  (`obstacles_inflation`) instead of the 0.1 m point-mass default used with the
  kinematic simulator, which has no collisions.

## Protocol

Two-frame ZeroMQ messages (`topic`, little-endian payload); layouts are in
[`star_nav_mujoco/protocol.py`](star_nav_mujoco/protocol.py) and mirrored in
[`mujoco_bridge/src/protocol.h`](../mujoco_bridge/src/protocol.h).

| Topic | Direction | Rate | Content |
| --- | --- | --- | --- |
| `state` | sim → bridge | 200 Hz | time, pose, velocity, angular rate, contacts |
| `depth` | sim → bridge | 30 Hz | time, optical-frame pose, intrinsics, uint16 mm image |
| `lidar` | sim → bridge | 10 Hz | time, sensor pose, float32 x y z per return (sensor frame) |
| `world` | sim → bridge | 1 Hz | obstacle primitives (RViz cloud, exact markers for tests) |
| `cmd` | bridge → sim | as published | position, velocity, acceleration, yaw, yaw rate |

## Performance

Measured on a laptop (i7-12700H, RTX 3050) for the default 150-obstacle world:

| Item | EGL (GPU) | OSMesa (software, Docker) |
| --- | --- | --- |
| Physics + controller step | ~80 µs | ~80 µs |
| Depth frame 640x480 | ~6 ms | ~17 ms |
| Real-time factor at 500 Hz / 30 Hz | 1.00 | 1.00 |

The GL backend is chosen automatically (EGL if available, else OSMesa);
override with `MUJOCO_GL=egl|osmesa|glfw`.

## Tests

```bash
cd mujoco_sim && .venv/bin/python -m pytest
```

They cover world generation, the wire protocol, hover, step response,
trajectory tracking, command timeout, collision detection and the depth
camera geometry (pixels back-projected through the published pose and
intrinsics must land on the ground plane and on the pillar surface).
