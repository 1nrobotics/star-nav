# MuJoCo Simulator: Architecture

How the MuJoCo simulator is organized: its layers, how it supports several
robots and sensors, and its main loop. It ends with the weak points and a
proposed cleanup (not implemented). For running it, see
[SIMULATION.md](SIMULATION.md) and [mujoco_sim/README.md](../mujoco_sim/README.md);
for the robots, [QUADRUPED.md](QUADRUPED.md) and [HUMANOID.md](HUMANOID.md).
Written November 2026, at commit 26f376c.

## Layers

```
┌─────────────── mujoco_sim/ (Python, no ROS) ────────────────────┐
│ __main__.py   command line → SimConfig, LidarConfig, RunConfig   │
│ sim.py        Simulator: scene, stepping, control, sensors       │
│               run(): real-time loop, ZeroMQ, viewer, recording   │
│ robots        quadrotor.py │ lite3.py │ t1.py                    │
│ scenes        world.py (forest) │ house.py │ scene.py (MJCF)     │
│               textures.py                                        │
│ sensors       lidar.py │ depth camera in sim.py                  │
│ wire format   protocol.py (in sync with mujoco_bridge protocol.h)│
└──────────────────────────────┬───────────────────────────────────┘
     ZeroMQ PUB: state, depth, lidar, world        SUB: cmd
┌──────────────────────────────┴───────────────────────────────────┐
│ mujoco_bridge/ (C++, ROS 1)                                      │
│   state → Odometry         depth → Image (16UC1) + camera_pose   │
│   lidar → PointCloud2 (world frame) + lidar_pose                 │
│   world → global_cloud, obstacles            pos_cmd → cmd       │
└──────────────────────────────┬───────────────────────────────────┘
┌──────────────────────────────┴───────────────────────────────────┐
│ star_nav_bringup/launch                                          │
│   exploration_mujoco.launch    quadrotor                         │
│   exploration_legged.launch    walking robots (robot:=lite3|t1)  │
│     ├ exploration_lite3.launch, exploration_t1.launch (wrappers) │
│     └ terrain_filter.py (lidar only)                             │
│   planning_stack.xml           EGO-Planner + exploration manager │
└──────────────────────────────────────────────────────────────────┘
```

Every robot has the same contract with the rest of the stack: it takes the
planner's position setpoint (`pos_cmd`: position, velocity, acceleration,
yaw) and returns odometry and sensor data. ROS and the planner do not know
which robot is simulated.

## Robots

### Registration

`sim.py`:

```python
ROBOTS = (QUADROTOR, LITE3, T1)
LEGGED = {LITE3: lite3, T1: t1}  # walking robot → its module
```

**The quadrotor** is a special case. Its body, camera (`depth`) and lidar
site are written by `scene.build_xml` (`quadrotor_body=True`). It is
controlled by `quadrotor.Controller`, a geometric SE(3) controller that
outputs four rotor thrusts.

**The walking robots** share a module convention. `lite3.py` and `t1.py`
define the same names; there is no base class or `Protocol`:

| Kind | Names | Purpose |
|---|---|---|
| Assets | `fetch()`, `check_assets()`, `asset_dir()` | Download model and policy at a pinned commit (never committed) |
| Model | `attach(spec, start, root, lidar)` | Attach the upstream MJCF with `MjSpec` under a prefix (`lite3/`, `t1/`); add the depth camera and the lidar housing; move collision geoms to `ROBOT_GROUP` |
| Constants | `TRUNK`, `CAMERA`, `TIMESTEP`, `START_HEIGHT`, `STAND_HEIGHT`, `LIDAR_MOUNT`, `LEG_GEOMS`, `HOUSE_WALL_HEIGHT` | Read by `Simulator` |
| Walking | `Walker` (`reset`, `policy_due`, `before_step`, `twist`) | Policy (NumPy) and joint PD |
| Steering | `Tracker.twist()` | Setpoint → (vx, vy, ωz); `t1.py` imports the Lite3's |

### Control chain per physics step

```
quadrotor:  cmd ─► quadrotor.Controller ─► 4 thrusts ─────────────────► mj_step
walking:    cmd ─► Tracker ─► twist ─► Walker (policy every N steps) ─►
                                       joint PD torques ──────────────► mj_step
```

`Simulator.step()` branches on `self.legged` into `step()` and
`_step_legged()`. Contact counting, fall detection and walkable surfaces
(ground, ramp, stairs: `house.WALKABLE`) are in `sim.py` too.
`set_command()` rejects a non-finite position and zeroes other non-finite
fields. When commands stop for `cmd_timeout` (0.5 s), the robot holds the
last position.

### Per-robot settings

Robot settings live in four places:

| Where | What |
|---|---|
| Robot module constants | Mounts, timestep, heights, leg geoms |
| `__main__._DEFAULTS` | Start, default world, rings |
| `scene.ROBOT_BODIES` | Body the viewer and recorder follow |
| `exploration_legged.launch` (`$(eval … if robot == 'lite3' else …)`) | Planning height, lidar mount and vertical FOV, terrain filter height, inflation, speed, RViz height |

## Sensors

| Sensor | Where | How | Rate | Message |
|---|---|---|---|---|
| Depth camera | `Simulator.render_depth()` | `mujoco.Renderer` off-screen (EGL, else OSMesa), 640×480, uint16 mm | 30 Hz | `depth`: image, intrinsics, optical-frame pose |
| Lidar | `lidar.Lidar` | `mj_multiRay` in chunks on a thread pool; directions fixed at construction (`beams × columns` grid) | 10 Hz | `lidar`: points in the sensor frame, sensor pose; the bridge transforms them to the world |
| State | `Simulator.state()` | Ground-truth `qpos`/`qvel`, contact count | 200 Hz | `state` |
| World | `world.py` obstacle list | Ground truth, for RViz and tests | 1 Hz | `world` |

- **Selection:** `--sensor depth|lidar|both`. Depth creates the renderer;
  lidar sets `SimConfig.lidar` to a `LidarConfig` built from
  `--lidar-beams/columns/vfov/rate/range/noise`. For walking robots, the
  mount comes from the module's `LIDAR_MOUNT`.
- **The robot does not see itself:** rays and the depth render skip
  `MARKER_GROUP` (2: setpoint, visual meshes) and `ROBOT_GROUP`
  (3: collision geoms).
- **To ROS:** the bridge's topics are remapped to the names the planner
  expects (`/drone_0_pcl_render_node/cloud`, …). The grid map needs the lidar
  mount and vertical FOV again (`lidar_offset`, `lidar_vfov` in the launch
  file).
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
  drain cmd messages → set_command
  every 1 s → send world;  every 1/30 s → sync the web viewer
  every 10 s → log rates and timings
```

Tests follow the layers:
- `mujoco_sim/tests/test_*.py`: pure Python, no ROS.
- `mujoco_smoke.test`, `lite3_smoke.test`, `t1_smoke.test`: the full
  ROS chain.

## Weak points

1. **Implicit robot interface.** `sim.py` branches on `self.legged`, and the
   quadrotor is built inside `scene.build_xml`. A new robot (a rover, say)
   means editing `sim.py`, `scene.py`, `__main__.py` and the launch files.
2. **Robot settings are spread over four places** (table above). The lidar
   mount exists twice, as `LIDAR_MOUNT` and as the launch file's
   `lidar_offset`, and the two must agree. Each new walking robot extends
   every `eval` line in `exploration_legged.launch`.
3. **`Tracker` lives in `lite3.py`**; `t1.py` imports it from there. There is
   no place for another tracker (MPPI, see
   [GROUND_PLANNING.md](GROUND_PLANNING.md) §6).
4. **Fixed sensor set:** one depth camera and at most one lidar.
   - The lidar has one scan pattern, a uniform grid.
   - The wire format has no sensor id and no per-point time, so two lidars
     or a motion-distorted scan need a protocol change.
   - A new scan pattern alone needs no protocol change.
5. **Sensors run in the physics thread.** Rendering and ray casting block
   stepping. An Airy-like 96 × 900 scan takes about 32 ms per frame in the
   forest (measured with 4 threads), a third of the 10 Hz budget.

## Review findings (November 2026)

From a read-through of the simulator, bridge, launch files, tests and CI.
These are about behaviour and test coverage; the weak points above are
about structure. Nothing here is fixed yet.

1. **Walking robots have no CI.** `sim-smoke.yml` runs the quadrotor only.
   `test_lite3.py` and `test_t1.py` skip without the downloaded assets, and
   `lite3_smoke` and `t1_smoke` are not in the workflow.
2. **The terrain filter's step limit ignores the robot.** `max_step` is 9 cm
   for both robots; `exploration_legged.launch` does not set it. The T1
   falls on 7 cm steps going up, yet the planner treats them as walkable.
3. **A fall is not reported.** It is logged only, and contact with the
   ground plane is never counted, so a fallen robot shows 0 contacts and
   the test fails on a later waypoint timeout
   ([HUMANOID.md](HUMANOID.md#falls-and-getting-up)).
4. **The terrain filter assumes clean data.** Each cell keeps its lowest
   return for ever, so one noisy return below the floor makes a permanent
   false obstacle. Lidar noise is 0 by default, which hides this. The
   filter has no unit tests.
5. **The house has no ground truth.** The world message carries pillars and
   rings only, so the house has no `global_cloud`, no obstacle markers and
   no map-accuracy check.
6. **Quadrotor names on shared code.** `quadrotor.Command` is the command
   type for every robot, and a `quadrotor.Controller` is built for walking
   robots too. The bridge logs "quadrotor collision", topics are
   `drone_0_*`, and `pyproject.toml` does not mention the T1.
7. **Smaller:**
   - When the viewer replaces `sim.data`, `Walker` keeps the old one (not
     verified that the viewer ever does this).
   - `receive()` in the bridge does not check the second `zmq_msg_recv`.
   - `_download` is duplicated in `lite3.py` and `t1.py`.

Suggested order: CI for the walking robots (download and cache the assets);
`max_step` per robot and unit tests for the terrain filter; the fall flag;
then the cleanup below.

## Proposed cleanup (not implemented)

Small, separately reviewable steps, in this order:

1. **Trackers out of `lite3.py`.** A `tracking.py` with the current
   proportional tracker, selected per robot or with `--tracker`. This is
   needed before an MPPI tracker, and changes no behaviour.
2. **Scan patterns for the lidar.** `LidarConfig.pattern`:
   `uniform` (today's, the default), `mid360`, `airy`.
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
3. **A robot interface.** A `Robot` protocol (attach, reset, control step,
   constants) implemented by all three robots, so `sim.py` loses its
   `legged` branches and the quadrotor leaves `scene.build_xml`.
4. **One source for per-robot settings.** A YAML file per robot, read by
   both the simulator and the launch files, replacing `_DEFAULTS`, the
   launch `eval` chains and the duplicated lidar mount.
5. **Lidar time (only if needed).** Cast the scan in slices during stepping,
   each with the pose at its time, and add per-point time to the wire format
   (`protocol.py`, `protocol.h`, `VERSION` bump). Star Nav uses the
   ground-truth pose, so nothing downstream needs this yet.

Each step that touches the simulator or bridge needs `pytest` in
`mujoco_sim/` and the smoke tests (`mujoco_smoke`, `lite3_smoke`,
`t1_smoke`), as [AGENTS.md](../AGENTS.md) requires.
