# GridMap PointCloud Raycasting Fix

## Problem Description

The exploration manager's occupancy grid was not detecting any free space when using pointcloud data from VK180, reporting:
- Unknown: 4489 cells
- **Free: 0 cells** ❌
- Occupied: 0 cells
- Result: No frontiers could be detected (frontiers exist at free/unknown boundaries)

## Root Cause Analysis

### GridMap Has Two Data Input Paths:

**1. Depth Image Path (Original Design):**
- Callback: `depthOdomCallback()` or `depthPoseCallback()`
- Process: Converts depth image → projects to 3D points → populates `md_.proj_points_[]`
- Raycasting: ✅ Enabled via `projectDepthImage()` + `raycastProcess()`
- Result: Properly marks free/occupied/unknown space

**2. PointCloud Path (Incomplete Implementation):**
- Callback: `cloudCallback()`
- Process: Receives 3D points → marks obstacles only
- Raycasting: ❌ **Was NOT enabled** - never populated `md_.proj_points_[]`
- Result: Only marked obstacles, never marked free space

### Why This Matters:

**EGO Planner (trajectory optimization):**
- Only needs: `getInflateOccupancy()` - checks if position is occupied
- Works with: ✅ Obstacle-only data from `cloudCallback`
- Doesn't need: Free/unknown distinction

**Exploration Manager (frontier detection):**
- Needs: `getOccupancy()` + `isUnknown()` + `isKnownFree()`
- Requires: ❌ Full raycasting to mark free space
- Frontiers: Found at **free/unknown boundaries** - impossible without free cells

## The Fix

Modified `cloudCallback()` in `/planner/plan_env/src/grid_map.cpp` to **optionally** enable raycasting:

### Changes Made:

**1. Added ROS parameter: `enable_pointcloud_raycasting` (default: false)**

```cpp
// In grid_map.cpp initMap()
node_.param("grid_map/enable_pointcloud_raycasting", mp_.enable_pointcloud_raycasting_, false);
```

**2. Conditional raycasting in cloudCallback()**

```cpp
// Only initialize if enabled
if (mp_.enable_pointcloud_raycasting_)
{
  md_.proj_points_cnt = 0;
}

// Only add points for raycasting if enabled
if (mp_.enable_pointcloud_raycasting_)
{
  double dist = devi.norm();
  if (dist > mp_.min_ray_length_ && dist < mp_.max_ray_length_)
  {
    md_.proj_points_[md_.proj_points_cnt++] = p3d;
  }
}

// Only trigger raycasting update if enabled
if (mp_.enable_pointcloud_raycasting_)
{
  md_.occ_need_update_ = true;
}
```

**3. Launch file configuration**

```xml
<!-- EGO Planner (planner/plan_manage/launch/advanced_param_exp.xml) -->
<!-- Parameter NOT set → defaults to false → original behavior preserved -->

<!-- Exploration Manager (vk180-ros/launch/vk180_exploration.launch) -->
<param name="grid_map/enable_pointcloud_raycasting" value="true" type="bool"/>
```

### How It Works Now:

```
PointCloud Data Flow:
┌─────────────────────────────────────────────────────────────┐
│ 1. cloudCallback() receives pointcloud                      │
│    - Populates md_.proj_points_[] with valid 3D points      │
│    - Marks obstacles in occupancy_buffer_inflate_           │
│    - Sets md_.occ_need_update_ = true                       │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 2. updateOccupancyCallback() timer triggers (20Hz)          │
│    - Calls projectDepthImage() (skips if no depth image)   │
│    - Calls raycastProcess()                                 │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 3. raycastProcess() performs raycasting                     │
│    - Iterates through md_.proj_points_[]                    │
│    - Casts rays from camera_pos_ to each point             │
│    - Marks free space along ray paths                       │
│    - Marks endpoints as occupied                            │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 4. Occupancy Map Updated                                    │
│    - Free cells: Space between camera and obstacles        │
│    - Occupied cells: Obstacle points + inflation           │
│    - Unknown cells: Not yet observed                        │
└─────────────────────────────────────────────────────────────┘
```

## Impact

### Configuration:

**EGO Planner:**
- Parameter: `enable_pointcloud_raycasting` = **false** (default)
- Behavior: ✅ **Unchanged** - uses original obstacle-only marking
- Map data: Obstacles in `occupancy_buffer_inflate_[]`, no raycasting
- Use case: Collision avoidance only needs obstacle positions

**Exploration Manager:**
- Parameter: `enable_pointcloud_raycasting` = **true** (explicitly set)
- Behavior: ✅ **Enhanced** - adds raycasting for free space detection
- Map data: Full occupancy map with free/unknown/occupied
- Use case: Frontier detection needs free/unknown boundaries

### Safety:

- ✅ **Backward compatible** - default behavior unchanged
- ✅ **Independent control** - each GridMap instance configures separately  
- ✅ **No interference** - EGO planner behavior unaffected
- ✅ **Selective enabling** - only exploration uses new feature

## Technical Details

### Ray Length Filtering:
```cpp
if (mp_.enable_pointcloud_raycasting_ && devi.norm() > mp_.min_ray_length_)
```
- `min_ray_length_`: points closer than this are ignored (sensor self-returns)
- Points beyond `max_ray_length_` are kept: `raycastProcess()` shortens them
  to a free-only ray of `max_ray_length_`. (Until 2026-11 they were dropped;
  see "Clearing Obstacles That Moved" below.)
- Points outside the local update box are ray-cast too; only the direct
  obstacle marking in `cloudCallback()` is limited to the box.

### Dual Functionality:
The `cloudCallback()` now performs **two roles**:
1. **Obstacle marking** (original): For EGO planner collision avoidance
2. **Raycasting preparation** (new): For exploration frontier detection

Both functions share the same pointcloud input but serve different planning needs.

## Testing Verification

Expected log output after fix:
```
[FrontierFinder] Map stats - Unknown: 3500, Free: 800, Occupied: 189, Frontier cells: 150
[FrontierFinder] Search took 12ms, found 3 frontier clusters
```

Key indicators of success:
- **Free > 0**: Raycasting is working
- **Frontier cells > 0**: Free/unknown boundaries detected
- **Frontier clusters > 0**: Valid exploration goals found

## Related Files

- `/planner/plan_env/src/grid_map.cpp` - Main fix location
- `/planner/plan_env/include/plan_env/grid_map.h` - GridMap class definition
- `/planner/exploration_manager/src/frontier_finder.cpp` - Uses free/unknown for frontiers
- `/planner/bspline_opt/src/bspline_optimizer.cpp` - Uses inflate_occupancy only

## Compatibility

This fix maintains **backward compatibility**:
- Depth image path: Unchanged, still works as before
- EGO planner queries: Still use `occupancy_buffer_inflate_[]`, unaffected
- Exploration queries: Now properly supported with raycasting

## Performance Considerations

- Raycasting a 32 x 1024 lidar scan (~13.5k returns) takes ~9 ms per scan
  (i7-12700H, one thread) since full rays are traced, ~3.5 ms before.
- Memory: `proj_points_[]` is sized for a 640x480 depth image and doubles when
  a cloud is larger.

## Clearing Obstacles That Moved (2026-11)

### Symptom

With `enable_pointcloud_raycasting` and the MuJoCo 360° lidar, an obstacle
that appeared and then went away (a dynamic obstacle, a person walking past)
stayed in the occupancy map indefinitely while the vehicle hovered, even
though the lidar kept seeing through the now empty space. Depth input did not
show this.

### Root causes

Four independent problems, each enough to leave voxels behind:

1. **Distant returns were dropped.** `cloudCallback()` only queued points
   closer than `max_ray_length` (4.5 m). A beam that hits a wall at 6 m is
   the evidence that the first 4.5 m are free, but it never reached
   `raycastProcess()`, which already knows how to shorten long rays to
   free-only rays (that is how depth pixels with no or far returns clear).
2. **Rays started at the body centre.** `odomCallback()` used the odometry
   position as the ray origin. The lidar sits elsewhere on the vehicle (6 cm
   above the centre in MuJoCo); at 3 m the cast ray is ~3 cm off the real
   beam, which is enough to pass through the neighbouring voxel and never
   touch the one the beam hit.
3. **RayCaster stepped along the wrong line.** `RayCaster::setInput()`
   computed the boundary crossings (`tMax`, `tDelta`) from the difference of
   the start and end *cell indices* instead of the real direction. For a
   nearly horizontal beam crossing one z boundary, the walk left the start
   z-layer at 27 % of the way instead of 47 %. On 200 000 random segments,
   86 % of rays skipped at least one cell they pass through (1.4 M cells in
   total). The ray grid was also aligned to world multiples of the
   resolution while the map grid starts at `ground_height` (−0.01 m), so
   cells near z boundaries were attributed to the wrong voxel.
4. **Rays stopped at the first already-visited voxel.** `raycastProcess()`
   breaks a ray when it reaches a voxel another ray cleared in the same
   update. Two rays sharing one voxel do not share the rest of the path, so
   the voxels behind it are skipped. With a dense depth image other pixels
   cover them; lidar beams are ~1.5° apart (one voxel at 4 m), and a steady
   vehicle skips the same voxels every scan.

### Fixes

| # | Change | Where |
| --- | --- | --- |
| 1 | Queue every point beyond `min_ray_length` for ray casting | `GridMap::cloudCallback()` |
| 2 | New parameter `grid_map/lidar_offset` `[x, y, z]`: sensor position in the odometry body frame, rotated with the odometry orientation and added to the ray origin. Default `[0, 0, 0]` (unchanged behaviour); `exploration_mujoco.launch` sets `[0, 0, 0.06]` | `GridMap::odomCallback()`, `advanced_param.xml`, `planning_stack.xml` |
| 3 | Exact voxel traversal (Amanatides & Woo): `tMax`/`tDelta` from the real direction; an axis that reached the end cell never steps again, so the walk always terminates. Rays are traced in map grid coordinates (relative to `map_origin_`) | `RayCaster::setInput()`/`step()`, `GridMap::raycastProcess()` |
| 4 | For point cloud input (`!flag_use_depth_fusion`) every ray is traced to the sensor; the depth path keeps the early exit | `GridMap::raycastProcess()` |

Fix 3 also changes the depth path (and the kinematic simulator): rays now
visit exactly the voxels they pass through.

### How it was measured

A script sat between the MuJoCo bridge and the planner (bridge output
remapped to `/lidar_raw`, the script republished every scan on
`/drone_0_pcl_render_node/cloud`). With the vehicle hovering at the start
point it picked an azimuth with returns at 6–9 m and, for 1 s, shortened all
returns within ±3° azimuth and ±5° elevation to 3 m along their beam, i.e. an
object appearing 3 m away exactly where real beams would hit it. Afterwards
the scans were passed through unchanged, and the script timed how long the
occupied voxels at the fake returns stayed in `grid_map/occupancy`.

The fake returns lie on the real beams, which start at the lidar (6 cm above
the body centre). Early runs placed them on lines from the body centre
instead; those hide problem 2 and are marked "body".

| Variant | Voxels left 10 s after the object vanished | Raycast time per scan |
| --- | --- | --- |
| Original code (body) | 16 of 16 | 1.9 ms |
| Fix 1 (body) | 6 | 3.8 ms |
| Fixes 1 + 4 (body) | 0 | 16.7 ms |
| Fixes 1 + 4 | some (never cleared) | 16.7 ms |
| Fixes 1 + 2 + 4 | 1 | – |
| Fixes 1 + 2 + 3, early exit kept | 6 of 11 | 3.5 ms |
| **All four** | **0, cleared 0.12 s after the object vanished (3/3 runs)** | 9 ms |

### Tests

- `catkin_make run_tests_plan_env`: `test/test_raycast.cpp` checks that
  RayCaster visits every cell of 2000 random segments plus axis-aligned,
  boundary and degenerate cases. The original RayCaster fails it.
- `rostest star_nav_bringup mujoco_smoke.test sensor:=lidar` and the depth and
  kinematic smoke tests pass unchanged (see docs/SENSORS.md for the map
  accuracy checks).

### Still open

- **Beams with no return** (sky, beyond the lidar range) carry no free-space
  information. An obstacle that leaves towards open space stays in the map.
  Treating a missing return as "free up to N m" would fix it in simulation,
  but real lidars also miss black, specular and glass surfaces, so this
  needs a per-sensor decision (e.g. only short ranges or only upward beams).
- **No automated dynamic-obstacle test.** The measurement above was done by
  hand; a moving obstacle in the MuJoCo world with a clearing-time check in
  `mujoco_smoke.test` would guard it.

## ROG-Map-Style Map Update (2026-11)

Three ideas from [ROG-Map](https://github.com/hku-mars/ROG-Map) (HKU MARS,
GPL-3.0; also the map of SUPER), re-implemented in this EGO-Planner **v1**
`grid_map` without copying its code, plus one bug fix. The external API the
planner uses (`getInflateOccupancy`, `getResolution`, `getRegion`,
`getOdomDepthTimeout`) is unchanged.

### 1. Hits per scan (`grid_map/max_hits_per_update`, default 1)

Each voxel gets one log-odds update per scan: a hit if at least as many rays
ended in it as passed through it (unchanged). New: the hit is weighted by the
number of rays that ended in the voxel, capped at `max_hits_per_update`.
With the defaults (`p_hit` 0.65, from unknown at `p_min` 0.12 to `p_occ`
0.80) a voxel needs 6 hits: 6 scans before, 1 scan for a surface with at
least 6 returns per voxel now. A single stray return still counts once. The
lidar launch uses 6; depth keeps 1.

### 2. Incremental inflation

`occupancy_buffer_inflate_` is now a counter per voxel: the number of
occupied voxels within the inflation distance (> 0 means occupied for the
planner). Every occupancy change goes through `setLogOdds()`, which adds or
removes the voxel's contribution when it crosses `p_occ`. This replaces the
per-update "clear the updated box and re-inflate every occupied voxel in it"
of `clearAndInflateLocalMap()` (that function now only forgets the margin
shell, as before). It also fixes stale inflation: a forgotten voxel used to
leave its inflated neighbours behind. The per-scan "replace" path
(`enable_pointcloud_raycasting` false) still writes the buffer directly.

### 3. Sliding map (`grid_map/sliding_map`, `sliding_size_x/y`, `sliding_threshold`)

Indices stay global (relative to `map_origin_`); the stored voxels are a
window `[window_min_, window_min_ + map_voxel_num_)` addressed as a ring
buffer (per-axis lookup tables, `toAddress`/`addressIfInMap`). When the
vehicle has moved `sliding_threshold` from the window centre,
`slideWindow()`:

1. takes back the inflation of occupied voxels that leave the window,
2. resets them to unknown (their memory is reused by the voxels that enter),
3. recomputes the inflation counters of the entering voxels from the occupied
   voxels already stored (`rebuildInflation`).

Only x and y slide; z covers the map height. Without `sliding_map` the window
is the whole map at the origin and the addresses are the old row-major ones.
Positions outside the window return -1 from `getInflateOccupancy()`, which
the planner treats as occupied; the window must cover the planning horizon.
`WaypointGenerator::isValidWaypoint` now rejects only occupied waypoints
(== 1), so waypoints beyond the window are kept.

### 4. Fix: raw points no longer painted in ray-casting mode

`cloudCallback()` ran the upstream "replace" path even with ray casting on:
it cleared the inflated map around the vehicle and painted every return of
the latest scan into it, until the next occupancy update (up to 50 ms later)
recomputed the box from the log-odds map. Obstacles flickered in for one scan
and out again until confirmed. In ray-casting mode the callback now only
queues the points.

### Not done: multi-resolution inflation

ROG-Map can keep the inflated map at a coarser resolution than the occupancy
map. With incremental counters the inflation is no longer a measurable cost
(the update is dominated by ray traversal), and the planner queries the map
at its resolution, so this was left out.

### Measurements

Replay of one recorded MuJoCo lidar mission (70 s, 32 x 1024 at 10 Hz,
~13 500 points and ~236 000 voxel visits per scan) into a standalone
grid_map node, same bag for every run, on an i7-12700H:

| | Old | New, `max_hits` 1, fixed map | New, `max_hits` 6, sliding 20 m |
| --- | --- | --- | --- |
| Process CPU for the replay | 17.5 s | 13.8 s | 14.2 s |
| Peak memory | 238 MB | 249 MB | 89 MB |
| Map update (timer callback), P-core | 6.6 ms | 12.0 ms | 12.6 ms |

The new code uses ~20% less CPU per scan, but its timer callback looks twice
as slow. That is not new work: both do exactly the same ray traversal
(counted), and the old code with only fix 4 applied also takes 12.0 ms and
13.9 s. The old callback ran right after `cloudCallback()` had painted every
return into the map, which left the caches warm for the ray loop; that paint
cost more (about 10 ms per scan) than it saved (about 5 ms). Per scan the
new code is ~5 ms cheaper from cloud arrival to updated map. The ray loop is
memory bound (a sampling profile puts the time on the first load of each
voxel's counters); packing the four per-voxel counters into one struct and
2 MB pages for the buffers were tried and made no difference.

MuJoCo smoke tests with the new defaults: lidar and depth 12/12 waypoints,
0 contacts, 0 phantom voxels, all nearby pillars mapped (lidar); kinematic
smoke test passes.

### Tests

`rostest plan_env test_grid_map.test` (`test/test_grid_map.cpp`, needs a ROS
master; `catkin_make run_tests_plan_env` runs it):

- incremental inflation equals a brute-force recount after random changes and
  after sliding (part-overlapping and disjoint windows);
- a dense surface is occupied after 6 scans with `max_hits` 1 and after 1
  with 6; a single stray return is not;
- an obstacle that moves away is cleared with its inflation;
- ray casting does not paint raw points; the replace mode still works;
- the sliding map maps a wall 20 m away (twice the configured map size) and
  forgets the one left behind.

Removing the entering-voxel rebuild in `slideWindow()` makes the brute-force
test fail.

---

**Author:** GitHub Copilot  
**Date:** 2025-12-17  
**Issue:** Exploration occupancy map showing zero free cells with pointcloud input  
**Fix:** Enable raycasting in cloudCallback by populating projection buffer
