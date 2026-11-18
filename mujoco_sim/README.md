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
| `--seed` | 1 | Forest layout |
| `--map-size X Y` | 50 50 | Forest extent (m) |
| `--pillars`, `--rings` | 100, 50 | Obstacle counts |
| `--start X Y Z` | 0 0 1 | Spawn and initial hover point |
| `--bind` | 127.0.0.1 | ZeroMQ address (`0.0.0.0` for a bridge on another host) |
| `--sensor` | depth | `depth`, `lidar` or `both` ([docs/SENSORS.md](../docs/SENSORS.md)) |
| `--depth-rate` | 30 | Depth camera rate (Hz) |
| `--lidar-*` | 32 x 1024, 10 Hz | Lidar beams, columns, vertical FOV, rate, range, noise |
| `--realtime-factor` | 1.0 | Simulated seconds per wall-clock second |
| `--no-viewer`, `--viewer-port` | off, 8080 | Web viewer |
| `--record FILE` | off | Write an MP4 from the viewer's follow camera, with lidar returns as dots (needs `ffmpeg`) |

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
- Lidar (`--sensor lidar`): 32 beams over −22.5° … +22.5°, 1024 columns,
  10 Hz, 0.3–10 m, mounted 6 cm above the body centre. Each revolution is
  cast at one instant with `mj_multiRay` in parallel threads; points are sent
  in the sensor frame with the sensor pose. Sensors ignore visual-only
  markers (geom group 2), such as the setpoint sphere.
- Unlike the kinematic simulator, both sensors also see the ground, so the
  planner's occupancy map contains the floor (below 0.45 m with inflation,
  well under the 1 m flight altitude), as it would with a real sensor. RViz
  hides map voxels below 0.45 m (`visualization_min_height`) so obstacles stay
  visible.
- Contacts: the number of physics steps in which the quadrotor touched
  anything, published by the bridge on `/mujoco/contacts`. The simulator log
  names the geoms and the position of each new contact.
- The quadrotor is 0.19 m from centre to prop tip, so
  `exploration_mujoco.launch` sets the planner's obstacle inflation to 0.3 m
  (`obstacles_inflation`) instead of the 0.1 m point-mass default used with the
  kinematic simulator, which has no collisions. With 0.2 m (1 cm to spare) a
  propeller grazed a pillar during frontier exploration.

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
