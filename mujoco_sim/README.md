# star_nav_mujoco

Standalone MuJoCo 4 simulator for Star Nav. It replaces the kinematic
simulator (`poscmd_2_odom` + `pcl_render_node` + `random_forest`) with
rigid-body dynamics, a flight controller, a GPU/OSMesa depth camera, collision
detection and a browser viewer. It does not use ROS. The robot is a
quadrotor, or Pollen Robotics' Microduck biped walking with its learned
policy (see [Microduck](#microduck)).

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
| `--robot` | quadrotor | `quadrotor` or `microduck` |
| `--fetch-microduck` | | Download the Microduck model and policy, then exit |
| `--seed` | 1 | Forest layout |
| `--map-size X Y` | 50 50 (duck: 12 12) | Forest extent (m) |
| `--pillars`, `--rings` | 100, 50 (duck: 60, 0) | Obstacle counts |
| `--start X Y Z` | 0 0 1 | Spawn and initial hover point (the duck ignores Z) |
| `--bind` | 127.0.0.1 | ZeroMQ address (`0.0.0.0` for a bridge on another host) |
| `--depth-rate` | 30 | Depth camera rate (Hz) |
| `--realtime-factor` | 1.0 | Simulated seconds per wall-clock second |
| `--no-viewer`, `--viewer-port` | off, 8080 | Web viewer |
| `--record FILE` | off | Write an MP4 from the viewer's follow camera (needs `ffmpeg`) |

The web viewer listens on all network interfaces (a MuJoCo Studio default
that cannot be changed yet); do not expose the port on untrusted networks.

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
- Unlike the kinematic simulator, the depth camera also sees the ground, so
  RViz shows the floor in the planner's occupancy map (bottom 0.1 m layer,
  well below the 1 m flight altitude), as it would with a real camera.
- Contacts: the number of physics steps in which the quadrotor touched
  anything, published by the bridge on `/mujoco/contacts`.
- The quadrotor is 0.19 m from centre to prop tip, so
  `exploration_mujoco.launch` sets the planner's obstacle inflation to 0.2 m
  (`obstacles_inflation`) instead of the 0.1 m point-mass default used with the
  kinematic simulator, which has no collisions.

## Microduck

[Microduck](https://github.com/pollen-robotics/microduck) is Pollen Robotics'
25 cm open-source biped. With `--robot microduck` it replaces the quadrotor:
same world generator, depth camera, protocol and planning stack, but it walks.

```bash
./scripts/setup_mujoco_env.sh --microduck   # onnxruntime, BAM, downloads the duck
roslaunch star_nav_bringup exploration_microduck.launch use_rviz:=true
rostopic pub -1 /exploration/trigger geometry_msgs/PoseStamped "{}"
rostest star_nav_bringup microduck_smoke.test   # headless, ~10 simulated minutes
```

How it works ([`star_nav_mujoco/microduck.py`](star_nav_mujoco/microduck.py)):

- **Model**: upstream `robot_groundcontact_backlash.xml` (the variant the
  policy is trained on, with 2° of gear backlash), attached to the forest scene
  with the `duck/` name prefix. Physics runs at 200 Hz.
- **Servos**: the XL330 servos are simulated with
  [BAM](https://github.com/Rhoban/bam) (firmware position loop, DC motor,
  voltage limit and friction), the actuator model the policy was trained with.
- **Walking**: `velstand.onnx` from
  [pollen-robotics/microduck-policies](https://huggingface.co/pollen-robotics/microduck-policies)
  runs at 50 Hz on gyro, gravity direction, joint positions and velocities,
  its last action and a command (forward speed, sideways speed, turn rate,
  head and body pose); its output is added to the standing pose as servo
  targets.
- **Steering**: the planner's setpoints are turned into commands by
  `Tracker`. The policy follows forward speed and turn rate but does not step
  sideways, does not start walking below a forward command of ~0.25 and turns
  in place only at a turn command of ~1.5 (measured: 0.69 · cmd − 0.09 m/s
  forward, 0.7 rad/s in place). So the duck is steered like a unicycle: it
  turns in place towards the setpoint when the heading error exceeds 50°,
  otherwise it walks with a forward command mapped through that line and a
  proportional turn rate. It stands still within 8 cm of a resting setpoint,
  and when it gets ahead of a setpoint that moves slower than it can walk
  (~0.1 m/s), it waits for it instead of turning back.
- **Depth camera**: the quadrotor's camera (640x480, fx = 387.2), placed at
  the duck's `head_camera` site, just outside the face shell, about 0.23 m
  above the floor. It moves with the head while walking. The duck's own
  visual meshes are hidden from it.
- **Contacts**: touching anything but the floor counts; a fall (trunk below
  60 % of its standing height) is logged.

`exploration_microduck.launch` scales the task to the duck: a 12 m forest of
5–30 cm pillars, 0.6–1.2 m tall (no rings), a 4.5 m lawnmower pattern with
1.5 m spacing at its trunk height (0.12 m), planner speed 0.2 m/s, 0.15 m
obstacle inflation, and `obstacle_min_height` 0.05 m so the floor the camera
sees is free space rather than an obstacle.

### Microduck files and licences

The duck's files are downloaded, not part of this repository, and pinned to
fixed upstream revisions (`MODEL_COMMIT`, `POLICY_REVISION` and the policy's
SHA-256 in `microduck.py`). They go to `~/.cache/star_nav/microduck`
(override with `STAR_NAV_MICRODUCK_DIR`):

| Files | Source | Licence |
| --- | --- | --- |
| `robot_groundcontact_backlash.xml` | pollen-robotics/microduck_rl | Apache-2.0 |
| `assets/*.stl` meshes | pollen-robotics/microduck_rl | **CC BY-NC-SA 4.0** (non-commercial) |
| `velstand.onnx` | pollen-robotics/microduck-policies (tag v10) | Apache-2.0 |
| BAM (`better-actuator-models`, pip) | Rhoban/bam | Apache-2.0 |

The meshes' non-commercial licence is why they are not redistributed here.
BAM declares `requires-python < 3.13` but is pure Python; the setup script
installs the commit the policy was trained with using `--ignore-requires-python`.

## Protocol

Two-frame ZeroMQ messages (`topic`, little-endian payload); layouts are in
[`star_nav_mujoco/protocol.py`](star_nav_mujoco/protocol.py) and mirrored in
[`mujoco_bridge/src/protocol.h`](../mujoco_bridge/src/protocol.h).

| Topic | Direction | Rate | Content |
| --- | --- | --- | --- |
| `state` | sim → bridge | 200 Hz | time, pose, velocity, angular rate, contacts |
| `depth` | sim → bridge | 30 Hz | time, optical-frame pose, intrinsics, uint16 mm image |
| `world` | sim → bridge | 1 Hz | obstacle primitives (rasterised into the RViz cloud) |
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
`tests/test_microduck.py` checks the duck's steering logic, and, when the
duck is downloaded, that it stands, walks to a setpoint, detects obstacle
contacts but not the floor, and that its head camera's returns all
back-project onto the floor or a pillar while walking.
