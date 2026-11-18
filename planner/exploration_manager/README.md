# exploration_manager

Waypoint exploration for the EGO planner: covers a rectangular area with a
lawnmower pattern and sends one goal at a time to the planner.

## State Machine

`INIT -> WAIT_TRIGGER -> GENERATE_WAYPOINTS -> PLAN_TO_WAYPOINT <-> EXEC_WAYPOINT -> FINISH -> WAIT_TRIGGER`

- `INIT` waits for odometry.
- `WAIT_TRIGGER` waits for `/exploration/trigger`.
- `GENERATE_WAYPOINTS` builds the lawnmower path, starting with three lead-in
  points that ease the approach to the first scan line. With `enable_corner_smoothing`, each 90
  degree corner is replaced by four intermediate points.
- `PLAN_TO_WAYPOINT` publishes the next goal on `/move_base_simple/goal`.
- `EXEC_WAYPOINT` waits until the vehicle is within 0.5 m of the goal, or
  has stopped within 1.5 m of it for 3 s (a waypoint next to an obstacle lies
  in the planner's inflated map, so the planner stops short of it), or
  skips it after `fsm/waypoint_timeout`.
- `FINISH` resets and returns to `WAIT_TRIGGER`.

## Topics

| Direction | Topic | Type |
| --- | --- | --- |
| Subscribe | `/mavros/local_position/odom` (remapped in simulation) | `nav_msgs/Odometry` |
| Subscribe | `/exploration/trigger` | `geometry_msgs/PoseStamped` |
| Subscribe | `/exploration/finish` | `std_msgs/Empty` |
| Publish | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` |
| Publish | `/exploration/state` | `std_msgs/String` |
| Publish | `/exploration/parameters` (latched) | `std_msgs/String` |

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `exploration/center_x`, `center_y` | 0.0 | Area centre (m) |
| `exploration/width`, `height` | 9.0 | Area size (m) |
| `exploration/altitude` | 1.0 | Flight altitude (m) |
| `exploration/line_spacing` | 3.0 | Distance between lawnmower lines (m) |
| `exploration/enable_corner_smoothing` | false | Round 90 degree corners |
| `fsm/waypoint_timeout` | 30.0 | Seconds before a waypoint is skipped |
| `fsm/stall_radius`, `fsm/stall_time` | 1.5, 3.0 | Stopped this close (m) for this long (s) counts as reached |
| `fsm/max_exploration_time` | 600.0 | Mission time limit (s) |

## Usage

The node is started by `star_nav_bringup/launch/exploration_sim.launch`:

```bash
roslaunch star_nav_bringup exploration_sim.launch use_rviz:=true
rostopic pub -1 /exploration/trigger geometry_msgs/PoseStamped "{}"
```

`launch/exploration.launch` is a standalone example and is not used by the
simulation.
