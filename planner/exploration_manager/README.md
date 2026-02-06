# Exploration Manager

Simple waypoint-based autonomous exploration for UAV navigation with EGO Planner.

## Overview

This package provides autonomous exploration capabilities for drones using:
- **Waypoint-based Exploration**: Simple lawnmower pattern in a rectangular area
- **EGO Planner Integration**: Uses existing trajectory planner for safe navigation
- **Hardware Compatible**: Works with depth sensors and PX4 flight controller

## Quick Start

### Simulation Mode
```bash
# Launch exploration in simulation
roslaunch ego_planner single_run_in_sim.launch

# Trigger exploration
rostopic pub /exploration/trigger geometry_msgs/PoseStamped "{}"
```

## Algorithm

The waypoint-based exploration uses a modified lawnmower pattern:

### Parameters
- `center_x`, `center_y`, `center_z`: Rectangle center and flight altitude
- `width`, `height`: Rectangle dimensions
- `line_spacing`: Distance between parallel scan lines
- `waypoint_timeout`: Maximum time to spend at each waypoint

### FSM States

1. **INIT**: Initialize and wait for odometry
2. **WAIT_TRIGGER**: Wait for user to trigger exploration
3. **GENERATE_WAYPOINTS**: Generate lawnmower pattern waypoints
4. **PLAN_TO_WAYPOINT**: Plan trajectory to next waypoint
5. **EXEC_WAYPOINT**: Execute trajectory and monitor progress
6. **FINISH**: Exploration complete

## Dependencies

### ROS Packages
- `roscpp`
- `pcl_ros`
- `plan_env` (from EGO planner)
- `plan_manage` (from EGO planner)
- `traj_utils` (from quadrotor_msgs)

### External Libraries
- Eigen3
- PCL (Point Cloud Library)

## Topics

### Subscribed
- `/mavros/local_position/odom` (nav_msgs/Odometry): Odometry
- `/exploration/trigger` (geometry_msgs/PoseStamped): Trigger exploration
- `/exploration/finish` (std_msgs/Empty): Manual finish command

### Published
- `/drone_0_planning/pos_cmd` (quadrotor_msgs/PositionCommand): Position commands
- `/exploration/state` (std_msgs/String): Current FSM state

## License

Same as parent workspace
