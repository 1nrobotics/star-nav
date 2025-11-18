# Lite3 Quadruped Controller

ROS package for controlling Deep Robotics Lite3 quadruped robot using EGO planner trajectories from VK180 sensor data.

## Overview

This package converts position/velocity/acceleration commands from the EGO planner into velocity commands suitable for the Lite3 quadruped robot. The controller implements:

- **Velocity feedforward**: Uses planned velocity from EGO planner
- **Position feedback**: Adds proportional control based on position error
- **Velocity smoothing**: Exponential moving average filter
- **Safety limits**: Max velocity limits and timeout protection
- **Height management**: Ignores Z-axis (Lite3 has built-in height controller)

## Architecture

```
VK180 Sensor → vk180_planner_bridge → /vk180/pointcloud, /vk180/odom_raw
                                            ↓
                                       EGO Planner
                                            ↓
                                   /drone_0_planning/pos_cmd
                                   (PositionCommand msg)
                                            ↓
                              Lite3 Velocity Controller
                                            ↓
                                     /lite3/cmd_vel
                                     (Twist msg)
                                            ↓
                                    Lite3 Robot SDK
```

## Message Flow

### Input: `quadrotor_msgs/PositionCommand`
```
position: {x, y, z}        # Desired position (m)
velocity: {x, y, z}        # Desired velocity (m/s)
acceleration: {x, y, z}    # Desired acceleration (m/s²)
yaw: float                 # Desired yaw angle (rad)
yaw_dot: float             # Desired yaw rate (rad/s)
```

### Output: `geometry_msgs/Twist`
```
linear: {x, y, z=0}        # Linear velocity command (m/s)
angular: {x=0, y=0, z}     # Angular velocity command (rad/s)
```

## Control Law

The velocity controller implements:
```
v_cmd = v_desired + Kp * (p_desired - p_current)
```

Where:
- `v_desired`: Velocity from planner (feedforward)
- `Kp`: Position gain (default 0.5)
- `p_desired - p_current`: Position tracking error

The output is then:
1. Smoothed with exponential moving average
2. Limited by max velocity constraints
3. Published to `/lite3/cmd_vel`

## Parameters

See `config/lite3_ctrl_params.yaml` for detailed parameter descriptions.

Key parameters:
- `max_linear_vel`: 1.0 m/s (safety limit)
- `max_angular_vel`: 1.0 rad/s (yaw rate limit)
- `position_gain`: 0.5 (tracking gain)
- `velocity_alpha`: 0.3 (smoothing factor)
- `control_rate`: 50 Hz

## Launch Files

### `lite3_autonomous_nav.launch`
Complete autonomous navigation setup for Lite3 with VK180 sensor.

**Usage:**
```bash
roslaunch lite3_ctrl lite3_autonomous_nav.launch
```

**Key arguments:**
- `use_rviz:=true` - Enable RViz visualization
- `flight_type:=1` - Use 2D Nav Goal (interactive)
- `flight_type:=2` - Use predefined waypoints
- `max_vel:=0.8` - Max velocity for planner (m/s)
- `max_linear_vel:=1.0` - Max velocity for controller (m/s)

## Usage Examples

### 1. Interactive Navigation (2D Nav Goal)
```bash
# Launch with RViz
roslaunch lite3_ctrl lite3_autonomous_nav.launch use_rviz:=true flight_type:=1

# In RViz: Use "2D Nav Goal" tool to set destination
```

### 2. Autonomous Waypoint Following
```bash
# Launch with predefined waypoints
roslaunch lite3_ctrl lite3_autonomous_nav.launch flight_type:=2

# Robot will follow waypoints defined in launch file
```

### 3. Monitor Velocity Commands
```bash
# View velocity commands being sent
rostopic echo /lite3/cmd_vel

# View trajectory commands from planner
rostopic echo /drone_0_planning/pos_cmd
```

## Safety Features

1. **Odometry Timeout**: Stops robot if no odometry for 0.5s
2. **Command Timeout**: Stops robot if no planner commands for 1.0s
3. **Velocity Limiting**: Enforces max velocity constraints
4. **Smooth Shutdown**: Sends zero velocity on node exit
5. **Velocity Smoothing**: Prevents jerky motion

## Tuning Guide

### Tracking is too sluggish
- Increase `position_gain` (e.g., 0.7-1.0)
- Decrease `velocity_alpha` (more responsive)

### Motion is too jerky
- Decrease `position_gain` (e.g., 0.3-0.4)
- Increase `velocity_alpha` (more smoothing)
- Reduce `max_vel` in planner

### Robot overshoots waypoints
- Decrease `position_gain`
- Reduce `max_vel` and `max_acc` in planner

### Need faster motion
- Increase `max_linear_vel` (up to ~1.5 m/s for Lite3)
- Increase `max_vel` in planner (keep below max_linear_vel)

## Integration with Lite3 SDK

This package publishes to `/lite3/cmd_vel`. You need to integrate with the actual Lite3 SDK to send these commands to the robot.

**Option 1: ROS Bridge** (if Lite3 has ROS support)
```bash
# The Lite3 SDK should subscribe to /lite3/cmd_vel
```

**Option 2: Custom Bridge Node**
Create a bridge node that:
1. Subscribes to `/lite3/cmd_vel`
2. Converts Twist messages to Lite3 SDK commands
3. Sends commands via Lite3 API

## Nodes

### `lite3_velocity_controller`
Converts trajectory commands to velocity commands.

**Subscribed Topics:**
- `/drone_0_planning/pos_cmd` (quadrotor_msgs/PositionCommand)
- `/vk180/odom_raw` (nav_msgs/Odometry) or `/mavros/local_position/odom`

**Published Topics:**
- `/lite3/cmd_vel` (geometry_msgs/Twist)

**Parameters:**
- See `config/lite3_ctrl_params.yaml`

## Dependencies

- `roscpp` / `rospy`
- `geometry_msgs`
- `nav_msgs`
- `quadrotor_msgs`
- `ego_planner`
- `vk180_ros`

## Building

```bash
cd ~/catkin_ws
catkin_make --pkg lite3_ctrl
source devel/setup.bash
```

## Troubleshooting

### No velocity commands published
- Check if planner is running: `rostopic hz /drone_0_planning/pos_cmd`
- Check if odometry available: `rostopic hz /vk180/odom_raw`
- Look for timeout warnings in controller logs

### Robot doesn't move
- Verify `/lite3/cmd_vel` has subscribers (Lite3 SDK bridge)
- Check velocity values are non-zero: `rostopic echo /lite3/cmd_vel`
- Ensure Lite3 is in velocity control mode

### Jerky motion
- Increase `velocity_alpha` for more smoothing
- Reduce `control_rate` if CPU limited
- Check odometry update rate is sufficient (>30 Hz)

## Future Improvements

- [ ] Add dynamic reconfigure for online parameter tuning
- [ ] Implement acceleration limits
- [ ] Add terrain-aware velocity adjustment
- [ ] Support for different gait speeds
- [ ] Emergency stop service
- [ ] Obstacle avoidance at controller level

## License

MIT

## Authors

1nrobotics Team
