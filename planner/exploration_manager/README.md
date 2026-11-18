# exploration_manager

Area exploration for the EGO planner. It sends one goal at a time on
`/move_base_simple/goal` and has two modes (`exploration/mode`):

- **`waypoint`** (default): flies a fixed lawnmower pattern over a rectangular
  area. It does not look at the map.
- **`frontier`**: explores the area's unknown space at flight height. It flies
  to the boundary between observed free space and unknown space until no
  reachable boundary is left, then returns to the start. It needs a map that
  observes free space: a depth camera, or a point cloud with ray casting. The
  MuJoCo simulator provides both; the kinematic simulator (`exploration_sim.launch`)
  provides neither, so frontier mode is not available there.

Both start on `/exploration/trigger` and publish their state on
`/exploration/state`.

## Frontier Mode

```
EGO grid_map ── grid_map/slice (2D, flight height) ──> FrontierMap ──> FSM ──> goal
```

1. **Map slice.** The planner's map publishes the voxels between
   `grid_map/slice_min_z` and `slice_max_z` around the sensor as a
   `nav_msgs/OccupancyGrid`, 5 times a second:

   | Cell value | Meaning |
   | --- | --- |
   | 100 | Any voxel in the band is occupied, or the voxel at mid-height is in the planner's inflated map |
   | 0 | At least half of the band's voxels are observed |
   | -1 | Otherwise unknown |

   The inflated map covers where the planner cannot fly, including near
   obstacles just above or below the band.

   `planning_stack.xml` sets the band to the altitude ± 0.25 m.
2. **Memory.** `FrontierMap` covers the area plus a margin. It keeps every
   observation, because the planner's sliding map forgets what lies outside
   its window. A later slice updates a cell, but an unknown cell in the slice
   never erases what is already known.
3. **Reachability.** Occupied cells are inflated again by
   `frontier/inflation`, a margin on top of the planner's inflation.
   Dijkstra (8-connected, no corner cutting) then runs from the robot over
   observed free cells. If the robot sits inside the inflation (close to an
   obstacle), it first searches up to `frontier/escape_radius` for free space.
4. **Frontiers.** A frontier cell is a reachable free cell inside the area
   with an unknown 4-neighbour inside the area. Three things are not
   frontiers:
   - unknown space outside the area, which is how exploration ends at the
     area's border;
   - single unknown holes;
   - space behind obstacles or gaps too narrow to pass.

   Cells are grouped into 8-connected clusters; clusters smaller than
   `frontier/min_cells` are dropped. Clusters longer than
   `frontier/max_cluster_size` are split. Each cluster's goal is its cell
   nearest the centroid.
5. **Goal selection.** The FSM runs at 4 Hz.

   **Cost of a frontier:**
   - its path length,
   - plus `turn_weight` × the heading change it needs,
   - minus `size_weight` × the cluster length (counted up to 2 m).

   **Keeping the current goal.** The goal stays until one of these happens:
   - the robot is within `reach_radius` of it (counts as reached);
   - no frontier is left within `goal_keep_radius` of it, i.e. it has been
     observed on the way;
   - another frontier is `switch_margin` cheaper.

   Goals change at most every `min_goal_interval`, unless one is reached or
   blocked: each new goal makes the planner start over.

   **Frontiers right next to the robot.** Frontiers closer than
   `min_goal_distance` are not flown to: a forward camera does not look
   there, and the planner handles very short hops poorly. If only such frontiers are left, the robot first backs off by
   `back_off` and then approaches them.
6. **Robustness.**
   - **No progress.** A goal with no progress (0.3 m) for `stuck_time` is
     blocked, together with frontiers within `blocked_radius`.
   - **Map not ready.** Without observed free space around the robot (the map
     has not arrived yet), the FSM waits instead of finishing.
   - **Finishing.** Exploration finishes after `finish_confirm` cycles in a
     row without a reachable frontier, or at `fsm/max_exploration_time`. The
     robot then returns to the start.

### States

`INIT -> WAIT_TRIGGER -> EXPLORE -> RETURN_HOME -> FINISH -> WAIT_TRIGGER`

### Topics

| Direction | Topic | Type |
| --- | --- | --- |
| Subscribe | `~slice` (remap to `.../grid_map/slice`) | `nav_msgs/OccupancyGrid` |
| Publish | `/exploration/coverage`: observed fraction of the area | `std_msgs/Float32` |
| Publish | `/exploration/map`: the memory; inflation shown as 50 (latched) | `nav_msgs/OccupancyGrid` |
| Publish | `/exploration/frontiers`: cells, cluster goals, current goal (latched) | `visualization_msgs/MarkerArray` |

### Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `frontier/resolution` | 0.1 | Cell size (m) |
| `frontier/inflation` | 0.3 | Distance kept from occupied cells (m); `planning_stack.xml` sets 0.15, a margin on top of the planner's inflation |
| `frontier/min_cells` | 8 | Smallest cluster (cells) |
| `frontier/max_cluster_size` | 3.0 | Longer clusters are split (m) |
| `frontier/escape_radius` | 1.0 | Search for free space from an inflated cell (m) |
| `frontier/goal_keep_radius` | 0.6 | The goal stays while a frontier is this close (m) |
| `frontier/switch_margin` | 2.0 | Cost a new frontier must save to replace the goal (m) |
| `frontier/turn_weight` | 1.0 | Cost per radian of heading change (m) |
| `frontier/size_weight` | 0.5 | Cost saved per metre of frontier (m/m) |
| `frontier/reach_radius` | 1.0 | A goal, or the start, is reached within this (m) |
| `frontier/min_goal_distance` | 2.0 | Nearer frontiers are not flown to (m) |
| `frontier/back_off` | 2.0 | Distance to back off from near frontiers (m) |
| `frontier/min_goal_interval` | 2.0 | Seconds between goal changes |
| `frontier/stuck_time` | 10.0 | Seconds without progress before a goal is blocked |
| `frontier/blocked_radius` | 1.0 | Frontiers this close to a blocked goal are skipped (m) |
| `frontier/finish_confirm` | 8 | Cycles (0.25 s) without frontiers before finishing |

## Waypoint Mode

`INIT -> WAIT_TRIGGER -> GENERATE_WAYPOINTS -> PLAN_TO_WAYPOINT <-> EXEC_WAYPOINT -> FINISH -> WAIT_TRIGGER`

- `INIT` waits for odometry.
- `WAIT_TRIGGER` waits for `/exploration/trigger`.
- `GENERATE_WAYPOINTS` builds the lawnmower path, starting with three lead-in
  points that ease the approach to the first scan line. With
  `enable_corner_smoothing`, each 90 degree corner is replaced by four
  intermediate points.
- `PLAN_TO_WAYPOINT` publishes the next goal on `/move_base_simple/goal`.
- `EXEC_WAYPOINT` waits until the vehicle is within 0.5 m of the goal, or
  skips it after `fsm/waypoint_timeout`.
- `FINISH` resets and returns to `WAIT_TRIGGER`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `exploration/line_spacing` | 3.0 | Distance between lawnmower lines (m) |
| `exploration/enable_corner_smoothing` | false | Round 90 degree corners |
| `fsm/waypoint_timeout` | 30.0 | Seconds before a waypoint is skipped |

## Common Topics and Parameters

| Direction | Topic | Type |
| --- | --- | --- |
| Subscribe | `/mavros/local_position/odom` (remapped in simulation) | `nav_msgs/Odometry` |
| Subscribe | `/exploration/trigger` | `geometry_msgs/PoseStamped` |
| Subscribe | `/exploration/finish` | `std_msgs/Empty` |
| Publish | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` |
| Publish | `/exploration/state` | `std_msgs/String` |
| Publish | `/exploration/parameters` (latched, waypoint mode) | `std_msgs/String` |

| Parameter | Default | Meaning |
| --- | --- | --- |
| `exploration/mode` | waypoint | `waypoint` or `frontier` |
| `exploration/center_x`, `center_y` | 0.0 | Area centre (m) |
| `exploration/width`, `height` | 9.0 | Area size (m) |
| `exploration/altitude` | 1.0 | Flight altitude (m) |
| `fsm/waypoint_tolerance` | 0.5 | Goal reached within this distance (m) |
| `fsm/max_exploration_time` | 600.0 | Mission time limit (s) |

## Usage

The simulation launch files start the node through
`star_nav_bringup/launch/planning_stack.xml`:

```bash
roslaunch star_nav_bringup exploration_mujoco.launch use_rviz:=true exploration_mode:=frontier
rostopic pub -1 /exploration/trigger geometry_msgs/PoseStamped "{}"

# Headless checks
rostest star_nav_bringup mujoco_smoke.test mode:=frontier
rostest star_nav_bringup mujoco_smoke.test mode:=frontier sensor:=lidar \
    exploration_width:=20 exploration_height:=20 mission_timeout:=400
```

`launch/exploration.launch` is a standalone example; the simulation does not
use it. The frontier map's unit tests:
`catkin_make run_tests_exploration_manager`.
