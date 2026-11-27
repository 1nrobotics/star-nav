# MuJoCo Simulator: Architecture

How the MuJoCo simulator is organized: its layers, how a robot plugs into
it, its sensors and its main loop. Then what is still weak, and the plan:
what has been done and what comes next. For running it, see
[SIMULATION.md](SIMULATION.md) and [mujoco_sim/README.md](../mujoco_sim/README.md);
for the robots, [QUADRUPED.md](QUADRUPED.md) and [HUMANOID.md](HUMANOID.md).
Written November 2026.

## Layers

```
┌─────────────── mujoco_sim/ (Python, no ROS) ────────────────────┐
│ __main__.py   command line → SimConfig, LidarConfig, RunConfig   │
│ sim.py        Simulator: scene, stepping, contacts, sensors      │
│               run(): real-time loop, ZeroMQ, viewer, recording   │
│ robots/       one module per robot: quadrotor, lite3, t1         │
│               __init__.py: the Robot interface and the registry  │
│ commands      commands.py (Setpoint, Velocity)                   │
│ shared by     tracking.py (setpoint → velocity command)          │
│ the robots    policy.py (MLP, GRU, ONNX weights, in NumPy)       │
│               assets.py (download, checksum, cache directory)    │
│               quadrotor.py (SE(3) flight controller)             │
│ scenes        world.py (forest) │ house.py │ scene.py (MJCF)     │
│               textures.py                                        │
│ sensors       lidar.py │ depth camera in sim.py                  │
│ wire format   protocol.py (in sync with mujoco_bridge protocol.h)│
└──────────────────────────────┬───────────────────────────────────┘
     ZeroMQ PUB: state, depth, lidar, world        SUB: cmd, vel
┌──────────────────────────────┴───────────────────────────────────┐
│ mujoco_bridge/ (C++, ROS 1)                                      │
│   state → Odometry         depth → Image (16UC1) + camera_pose   │
│   lidar → PointCloud2 (world frame) + lidar_pose                 │
│   world → global_cloud, obstacles            pos_cmd → cmd       │
│                                              cmd_vel → vel       │
└──────────────────────────────┬───────────────────────────────────┘
┌──────────────────────────────┴───────────────────────────────────┐
│ star_nav_bringup/launch                                          │
│   exploration_mujoco.launch    quadrotor                         │
│   exploration_<robot>.launch   a ground robot's sizes, then:     │
│     exploration_ground.launch  what ground robots share          │
│   planning_stack.xml           planner + exploration manager     │
│     planner.xml                the robot type's planner:         │
│     └ planners/ego.xml         aerial (the quadrotor)            │
│     └ planners/cmu.xml         legged (Lite3, T1)                │
└──────────────────────────────────────────────────────────────────┘
```

Every simulated robot has the same contract with the rest of the stack: it
takes a position setpoint (`pos_cmd`: position, velocity, acceleration,
yaw; `commands.Setpoint` in the simulator) and returns odometry and sensor
data. A ground robot also takes a body-frame velocity (`cmd_vel`: forward,
sideways, turn rate; `commands.Velocity`), which goes to its walking policy
within the tracker's limits and replaces the position setpoint once one has
been received; without a new one for `cmd_timeout` the robot stands. The planners use one each: EGO sends the quadrotor
setpoints, and the CMU local planner sends the walking robots a velocity
(their setpoint tracker is still in the simulator, but no planner sends
them setpoints now). Which planner a robot gets, and what a planner takes and gives, is in
[PLANNER_INTERFACE.md](PLANNER_INTERFACE.md).

## Robots

### The interface

Each module in `star_nav_mujoco/robots/` describes one robot with an object
named `ROBOT`. `Simulator` talks only to that object; it has no code for a
particular robot. `robots/__init__.py` defines the interface:

| `Robot` | Purpose |
|---|---|
| `name`, `title` | `--robot` value (also the module and the asset directory); name in messages |
| `body`, `camera` | Root body (a free joint) and depth camera in the compiled model |
| `on_ground` | Touching the ground is not a contact |
| `world`, `rings`, `start`, `lidar_vfov` | Defaults of the command line |
| `lidar_mount`, `lidar_pitch`, `follow_camera`, `house_wall_height` | Lidar position on the body and nose-down tilt by lidar model; viewer and recorder camera; wall height in the house |
| `check_assets()`, `fetch(log)` | Files downloaded at a pinned commit, never committed (`assets.py`) |
| `build(cfg, obstacles, extra)` | Returns the scene (`MjSpec`) with the robot in it |
| `controller(model, data, cfg)` | Returns a `Controller`: `accepts` (the commands of `commands.py` it follows), `reset()`, and `step(cmd)` before every `mj_step` |

The registry is the directory: `robots.names()` lists the modules,
`robots.get(name)` returns a module's `ROBOT`, and `robots.find(model)`
returns the robot in a compiled scene. Listing the robots does not import
MuJoCo, because the command line needs the names before it has chosen an
OpenGL backend.

**The quadrotor** (`robots/quadrotor.py`) is written in MJCF in its own
module: `parts()` gives its body, rotors, materials and default class, and
`scene.build_xml` puts them into the scene. It is flown by
`quadrotor.Controller`, a geometric SE(3) controller that outputs four rotor
thrusts, with the robot's `params` and `gains`. Its controller accepts
setpoints only; a velocity sent to it is refused and counted.

**Ground robots** subclass `GroundRobot`, which implements `build` and
`controller` from four things the robot gives:

| `GroundRobot` | Purpose |
|---|---|
| `attach(spec, start, root, lidar)` | Attach the upstream MJCF with `MjSpec` under a prefix (`lite3/`); add the depth camera and the lidar housing; move collision geoms to `ROBOT_GROUP` |
| `locomotion(model, data, root)` | The policy and joint control: `twist`, `policy_due`, `before_step()`, `reset()` |
| `tracker` | A class of `tracking.py`: `Holonomic` (Lite3, T1) |
| `timestep`, `stand_height`, `start_height`, `support_geoms` | Physics step of the policy; body height standing and when placed; the geoms it stands on |

### Control chain per physics step

```
              ┌──────────── Controller.step(cmd) ────────────┐
quadrotor:    cmd ─► quadrotor.Controller ─► 4 thrusts ───────► mj_step
ground robot: cmd ─► tracker ─► twist ─► locomotion policy
                                (every N steps) ─► joint torques ► mj_step
```

`Simulator.step()` is one loop for every robot. Contact counting, fall
detection (`Simulator.fallen`) and walkable surfaces (ground, ramp, stairs:
`house.WALKABLE`) are in `sim.py`. `set_command()` rejects a non-finite
position and zeroes other non-finite fields. `set_velocity()` asks the
controller (`accepts`), not the kind of robot, whether a velocity can be
followed. When commands stop for `cmd_timeout` (0.5 s), the robot holds the
last position.

The tracker is the seam for a better local controller: an MPPI tracker
([GROUND_PLANNING.md](GROUND_PLANNING.md) §6) is another class with the
same `twist()` method.

### Per-robot settings

| Where | What |
|---|---|
| `robots/<robot>.py` | Everything the simulator needs: model, policy, mounts, heights, lidar beam elevations, command-line defaults |
| `exploration_<robot>.launch` | What the planner needs: standing height, lidar mount, the height above which returns are overhead, speed; for frontier exploration the step limit and clearance |

Two numbers are in both: the standing height (`altitude` in the launch
file) and the lidar mount (`lidar_offset`). `tests/test_robots.py` reads
the launch files and fails if they differ.

A single YAML file per robot, read by both sides, was the first idea. It
does not work with ROS 1: `roslaunch` cannot read a file into `<arg>`
values (`$(eval)` has no file access, and args cannot come from the
parameter server). The launch file is therefore the planner-side half of the
description, and the test keeps the two halves in agreement.

### Adding a robot

1. `robots/<name>.py`: constants, `fetch`, `attach`, a `Locomotion` class
   and a `GroundRobot` subclass; `ROBOT = ...`. Use `assets.py` for the
   downloads and `policy.py` for the network.
2. `exploration_<name>.launch` with its sizes, and `<name>_smoke.test`.
3. `tests/test_<name>.py`; `test_robots.py` covers the descriptor and the
   launch file without changes.
4. One line each in the README table and [LICENSES.md](LICENSES.md).

Nothing in `sim.py`, `scene.py`, `__main__.py`, `record.py`, `viewer.py` or
`scripts/setup_mujoco_env.sh` changes: `--robot <name>`, `--fetch <name>`
and `setup_mujoco_env.sh --<name>` come from the registry.

## Sensors

| Sensor | Where | How | Rate | Message |
|---|---|---|---|---|
| Depth camera | `Simulator.render_depth()` | `mujoco.Renderer` off-screen (EGL, else OSMesa), 640×480, uint16 mm | 30 Hz | `depth`: image, intrinsics, optical-frame pose |
| Lidar | `lidar.Lidar` | `mj_multiRay` in chunks on a thread pool; directions fixed at construction (`beams × columns` grid) | 10 Hz | `lidar`: points in the sensor frame, sensor pose; the bridge transforms them to the world |
| State | `Simulator.state()` | Ground-truth `qpos`/`qvel`, contact count | 200 Hz | `state` |
| World | `world.py` obstacle list | Ground truth, for RViz and tests | 1 Hz | `world` |

- **Selection:** `--sensor depth|lidar|both`. Depth creates the renderer;
  lidar sets `SimConfig.lidar` to a `LidarConfig` built from
  `--lidar-beams/columns/vfov/rate/range/noise`. The mount and the default
  beam elevations come from the robot (`lidar_mount`, `lidar_vfov`).
- **The robot does not see itself:** rays and the depth render skip
  `MARKER_GROUP` (2: visual meshes) and `ROBOT_GROUP`
  (3: collision geoms).
- **To ROS:** the bridge's topics are remapped to the names the planner
  expects (`/drone_0_pcl_render_node/cloud` for the quadrotor,
  `/drone_0_lidar/cloud` for the ground robots). The grid map needs the lidar
  mount again (`lidar_offset` in the robot's launch file).
- **Lidar timing:** a whole revolution is cast at one instant (no motion
  distortion, no per-point time). The wire format carries `beams` and the
  points only.

## Main loop (`sim.run`)

Single-threaded, paced by the wall clock:

```
while True:
  target = sim time the wall clock asks for (resync if > 0.25 s behind)
  while sim.time < target:
      sim.step()                                   # 2 ms
      every state_every steps → send state
      depth due               → render, send       (blocks physics)
      lidar due               → cast, send         (blocks physics; threads)
      recorder captures if due
  drain cmd and vel messages → set_command, set_velocity
  every 1 s → send world;  every 1/30 s → sync the web viewer
  every 10 s → log rates and timings
```

Tests follow the layers:
- `mujoco_sim/tests/test_*.py`: pure Python, no ROS.
- `mujoco_smoke.test`, `lite3_smoke.test`, `t1_smoke.test`: the full ROS
  chain.

## Weak points

1. **Fixed sensor set:** one depth camera and at most one lidar.
   - The lidar has one scan pattern, a uniform grid.
   - The wire format has no sensor id and no per-point time, so two lidars
     or a motion-distorted scan need a protocol change.
   - A new scan pattern alone needs no protocol change.
2. **Sensors run in the physics thread.** Rendering and ray casting block
   stepping. An Airy-like 96 × 900 scan takes about 32 ms per frame in the
   forest (measured with 4 threads), a third of the 10 Hz budget.
3. **Results are pass or fail.** A smoke test says whether one mission on
   one seed succeeded. It does not say how often it succeeds, how long it
   takes, or how that changes when the terrain, the sensor noise or the
   robot's speed change. Comparing two trackers or two policies needs that
   (see the evaluation layer below).
4. **The planner-side description is a launch file** (see above); the test
   only checks the two numbers that exist on both sides.

## Review findings (November 2026)

From a read-through of the simulator, bridge, launch files, tests and CI.

1. **Ground robots had no CI.** `sim-smoke.yml` runs the quadrotor only.
   `test_lite3.py` and `test_t1.py` skip without the
   downloaded assets, and their smoke tests are not in the workflow. Since
   then tests run locally (`scripts/run_local_tests.sh`), the walking
   robots included; the workflow is started by hand only.
2. **The terrain filter's step limit is too high for the T1.** `max_step`
   is now set per robot (Lite3 and T1 9 cm), but the T1 falls on
   7 cm steps going up, yet the planner treats them as walkable. (The
   filter went with the walking robots' EGO path; the CMU planner's
   obstacle height, 0.15 m, has the same gap.)
3. **A fall is not reported.** `Simulator.fallen` is set and logged, but it
   is not in the state message, and contact with the ground plane is never
   counted. A fallen robot shows 0 contacts and the test fails on a later
   waypoint timeout ([HUMANOID.md](HUMANOID.md#falls-and-getting-up)).
4. **The terrain filter assumes clean data.** Each cell keeps its lowest
   return for ever, so one noisy return below the floor makes a permanent
   false obstacle. Lidar noise is 0 by default, which hides this. The
   filter has no unit tests. (The filter has been removed; the frontier
   map keeps a cell's lowest return the same way.)
5. **The house has no ground truth.** The world message carries pillars and
   rings only, so the house has no `global_cloud`, no obstacle markers and
   no map-accuracy check.
6. **Quadrotor names on shared code.** `quadrotor.Command` was the command
   type for every robot, `SimConfig` and `scene.build_xml` carried the
   quadrotor's parameters for every robot, and the bridge logged "quadrotor
   collision". Done: the commands are `commands.Setpoint` and
   `commands.Velocity`, the parameters and the MJCF are in the quadrotor's
   module, and the bridge logs "robot collision". Topics are still
   `drone_0_*`: that prefix is part of the planner interface
   ([PLANNER_INTERFACE.md](PLANNER_INTERFACE.md#topics)).
7. **Smaller:**
   - When the viewer replaces `sim.data`, the controller keeps the old one
     (not verified that the viewer ever does this).
   - `receive()` in the bridge does not check the second `zmq_msg_recv`.

## Plan

The aim is that the simulator grows in three directions without its core
changing: more robots, better controllers, and measured results.

### Done

1. **Shared modules.** `tracking.py` (the trackers, out of the robot
   modules), `assets.py` (one download and checksum function, one cache
   layout) and `policy.py` (MLP, GRU, gravity direction, ONNX weights).
2. **One description per robot.** The `Robot` object replaces
   `__main__._DEFAULTS`, `scene.ROBOT_BODIES`, `scene.follow_camera` and the
   module-name convention. The launch files' `$(eval dict(...)[robot])`
   lookups became plain arguments set by each robot's launch file.
3. **A robot interface.** `Simulator` has no `legged` branches; the
   quadrotor is a robot like the others.
4. **Neutral commands.** `commands.py` has the two commands a planner
   sends, and a controller says which it follows. `SimConfig` and
   `scene.py` know nothing of the quadrotor: its parameters, gains and MJCF
   are in `robots/quadrotor.py`. The physics did not change: the same
   commands give the same states, step for step, for all three robots.

### Next

5. **The fall flag on the wire.** Add `fallen` to the state message
   (`protocol.py`, `protocol.h`, `VERSION` bump), publish it from the
   bridge, and make the smoke test fail on it at once (finding 3).
6. **An evaluation layer.** A scenario is a world, a mission, a set of
   perturbations and a seed; a run returns a result record (JSON): success,
   time, path length, contacts, falls, minimum clearance. The smoke tests
   are the lowest tier of this. It sits beside `sim.py` and needs no ROS
   for controller-only scenarios (walk a line, climb the steps); full
   missions go through the launch files.

   | Tier | Perturbations |
   |---|---|
   | T0 | none: today's smoke tests |
   | T1 | seeds: forest layout, start pose |
   | T2 | sensing: lidar noise and dropouts, pose noise and delay |
   | T3 | dynamics: friction, payload, motor strength, pushes |
   | T4 | terrain: step height, ramp angle, clutter |

   Two variants (two trackers, say) run on the same seeds, so the comparison
   is paired and the answer is better, worse or not distinguishable, with a
   veto on any safety regression (falls, contacts).
7. **Lidar models.** Done for `uniform` and `mid360`: one class per sensor
   in `lidar_models.py` (beam directions, beams per scan, range), chosen with
   `--lidar-model` (`lidar:=` in the launch files); the robot gives the mount
   position and tilt. Still open: `airy`, and more than one sensor at a time.
   - `mid360`: Livox's measured pattern (`scan_mode/mid360.csv` from
     [livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation),
     MIT, 16.8 MB). Download it at setup, never commit it, and update
     [LICENSES.md](LICENSES.md). Use 20,000 consecutive directions per frame
     to reproduce the non-repetitive scan; elevations span −7.2° … 52.2°.
   - `airy`: no published beam table found; 96 beams evenly over 0–90°,
     an approximation.
   - Measured cost per frame: Mid-360 9.9 ms (forest) and 1.6 ms (house);
     current grid 14.2 ms and 3.0 ms.
   - No protocol change.
8. **`skills/`.** Short task guides for coding agents and contributors (add
   a robot, add a scenario, compare two variants), each ending in a command
   whose output shows the task is done.
9. **Lidar time (only if needed).** Cast the scan in slices during stepping,
   each with the pose at its time, and add per-point time to the wire format.
   Star Nav uses the ground-truth pose, so nothing downstream needs this yet.

### What we take from the Antigravity Robotics plugin

[robotics-antigravity-plugin](https://github.com/google-deepmind/robotics-antigravity-plugin) is a
set of agent skills for MuJoCo work. It is not a dependency here; three of
its ideas shaped the plan above:

| Its skill | What we take |
|---|---|
| `sim2real-gauntlet` | The evaluation layer: perturbation tiers, paired runs on common seeds, a three-way verdict with a safety veto |
| `iteration` | A change is done when a measurement says so, not when it runs: every step above ends in a test or a result record |
| Skill format | `skills/`: one short guide per task, with the command that verifies it |

Its `mujoco-audit` checks are written for tabletop manipulation assets and
do not fit downloaded robot models. Its `mujoco-playground-rl` skill (train
a policy for a Menagerie robot; needs a GPU) is the route to policies the
upstream ones lack: climbing 7 cm steps with the T1, getting up after a
fall.

Each step that touches the simulator or bridge needs `pytest` in
`mujoco_sim/` and the smoke tests (`mujoco_smoke`, `lite3_smoke`,
`t1_smoke`), as [AGENTS.md](../AGENTS.md) requires.
