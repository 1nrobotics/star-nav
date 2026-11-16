# Obstacle Sensors: Depth Camera or 360° Lidar

The MuJoCo simulator can give the planner either a forward depth camera (the
default, same as the kinematic simulator) or a 360° multi-beam lidar. Pick one
per launch:

```bash
roslaunch star_nav_bringup exploration_mujoco.launch use_rviz:=true                # depth camera
roslaunch star_nav_bringup exploration_mujoco.launch use_rviz:=true sensor:=lidar  # 360° lidar
```

Both feed the same EGO-Planner occupancy map (`grid_map`), but through
different code paths with different behaviour. This page explains the
differences, how the simulated data reaches the planner, and how its accuracy
is checked.

## Data Flow

```
depth:  MuJoCo depth render (640x480, 30 Hz) ──► bridge ──► /drone_0_pcl_render_node/depth (16UC1, mm)
                                                        └─► /drone_0_pcl_render_node/camera_pose
        grid_map: pairs image + pose by timestamp, back-projects every 2nd pixel with
        the intrinsics, ray-casts each pixel into the probabilistic map

lidar:  MuJoCo ray casting (32 x 1024, 10 Hz) ──► bridge ──► /drone_0_pcl_render_node/cloud (world frame)
        points in the sensor frame + sensor pose      └─► /drone_0_lidar/pose
        bridge transforms every point with the pose at scan time (like a
        lidar-inertial odometry "registered" cloud); grid_map ray-casts each
        point from the vehicle position into the same probabilistic map
```

## Main Differences

| | Depth camera | 360° lidar |
| --- | --- | --- |
| Field of view | 79° x 64°, forward only | 360° x 45° (−22.5° … +22.5°) |
| Samples | 640 x 480 at 30 Hz (planner uses every 2nd pixel) | 32 beams x 1024 columns at 10 Hz, ~13 k returns per scan |
| Resolution at 5 m | ~1.3 cm per pixel | 0.35° x 1.45°: ~3 cm horizontally, ~13 cm between beams |
| Range used | 0.2 … 5 m (`depth_filter_*`), ray-cast up to 4.5 m | 0.3 … 10 m, ray-cast up to 4.5 m |
| No return | pixel value 0 is treated as free space up to `max_ray_length` | no point at all: the ray carries no free-space information |
| Message | image + camera pose; grid_map projects with `cx, cy, fx, fy` | point cloud already in the world frame |
| Ray origin in grid_map | camera optical centre from `camera_pose` | vehicle position from odometry (the sensor is 6 cm above it) |
| What the planner knows | only what the camera has looked at; turning the yaw reveals more | everything around the vehicle within the beam fan |
| Blind spots | everything outside the forward frustum | above +22.5° and below −22.5° elevation, i.e. close below and above the vehicle; gaps between beams for thin horizontal structures |
| Ground | seen from ~1.6 m ahead | seen in a ring from ~2.6 m around the vehicle |

Notes:

- **Map update.** With `sensor:=lidar` the launch file sets
  `grid_map/enable_pointcloud_raycasting`, so each scan goes through the same
  log-odds update as depth (hits mark occupied, the voxels along each ray are
  cleared). A voxel needs several consistent hits before it counts as
  occupied, which filters noise. With `lidar_raycasting:=false` grid_map uses
  its upstream point-cloud path instead: each cloud *replaces* the local map
  (no memory, no clearing). See "Choosing the map update" below.
- **Motion distortion.** A real spinning lidar takes 100 ms per revolution, so
  at 1.5 m/s points at the start and end of a scan are 15 cm apart unless the
  driver or odometry de-skews them. The simulator casts the whole revolution at
  one instant, which corresponds to a perfectly de-skewed scan.
- **Noise.** Both sensors are exact by default. `--lidar-noise SIGMA` (via
  `sim_args`) adds Gaussian range noise to the lidar.
- **Point cloud frame.** grid_map expects world-frame points. With real
  hardware this is the registered cloud of a lidar-inertial odometry (e.g.
  FAST-LIO), not the raw driver output.

## Choosing the Map Update

`lidar_raycasting` selects how grid_map turns lidar scans into the map:

| | Ray-cast (default) | Replace per scan (`lidar_raycasting:=false`) |
| --- | --- | --- |
| Update | log-odds, same as depth: hits raise, rays lower | local box cleared, every point of the current scan marked occupied |
| Memory | keeps what left the field of view (e.g. right below the vehicle) | none: only the latest scan |
| Free space | cleared along every ray | not represented |
| Noise / stray returns | filtered: a voxel needs ~6 consecutive hits | every return is an obstacle at once |
| Latency for a new obstacle | ~0.6 s (6 scans at 10 Hz; 0.9 m at 1.5 m/s) | one scan (0.1 s) |
| Mission on seed 1 (≥ 3 runs each) | 12/12 waypoints, 0 contacts, 0 phantom voxels, all nearby pillars mapped | same |

In this static forest both work equally well. Ray-casting is the default
because it gives the map the same meaning as with the depth camera, so the
two sensors can be compared directly, and because it tolerates noisy returns
and blind zones the way a real lidar needs. The latency is safe here because
obstacles enter the map while still more than 3.5 m away (rays are used up to
4.5 m). Use the per-scan mode to study a lower-latency, memoryless map.

## Lidar Options

Simulator flags, passed with `sim_args:="..."`:

| Flag | Default | Meaning |
| --- | --- | --- |
| `--lidar-beams` | 32 | Number of beams (rows) |
| `--lidar-vfov LOW HIGH` | -22.5 22.5 | Elevation of the lowest and highest beam (deg) |
| `--lidar-columns` | 1024 | Azimuth samples per revolution |
| `--lidar-rate` | 10 | Scans per second |
| `--lidar-range` | 10 | Maximum range (m) |
| `--lidar-noise` | 0 | Range noise standard deviation (m) |

The sensor sits 6 cm above the body centre, aligned with the body (x forward,
z up). Rays ignore the quadrotor itself and visual-only markers such as the
blue setpoint sphere. Casting runs in parallel threads: ~7 ms per scan on 4
cores of an i7-12700H, ~13 ms while the planner runs.

## How Accuracy Is Checked

Three layers, all automated:

1. **Sensor geometry (pytest, `mujoco_sim/tests`).** With the body tilted and
   turned, every lidar return, transformed with the published sensor pose,
   lies within 2 mm of an analytic surface (ground, cylinder or ring capsule);
   returns respect the range and field of view; rays ignore the quadrotor and
   the setpoint marker. Depth pixels back-project onto the ground and pillar
   surfaces the same way.
2. **What the planner receives (`mujoco_smoke.test`).** The bridge publishes
   the exact obstacle geometry on `/mujoco/obstacles`. During the mission the
   test compares (`star_nav_bringup/test/map_accuracy.py`):
   - each lidar point on the planner's cloud topic against the nearest
     surface: any point farther than 2 cm fails the test (this checks the
     bridge's frame transform and timing end to end);
   - every occupied voxel of the planner's inflated map against the nearest
     obstacle: voxels farther than the inflation plus one voxel are phantoms
     (more than 1 % fails);
   - with the lidar, every pillar taller than the flight height that comes
     within 2.5 m of the quadrotor must appear in the map at flight height
     (at least 95 %).
3. **Flight.** The mission must reach all 12 waypoints with zero contacts.

Typical results on the default world (seed 1):

| | Depth camera | 360° lidar |
| --- | --- | --- |
| Sensor points checked | (image, checked in pytest) | ~220 000, max error 4.7 mm |
| Phantom voxels | 0 of ~255 000 sampled | 0 of ~255 000 sampled |
| Nearby pillars in the map | 8 of 11 (outside the camera view otherwise) | 11 of 11 |
| Mission | 12 waypoints, 0 contacts | 12 waypoints, 0 contacts |
