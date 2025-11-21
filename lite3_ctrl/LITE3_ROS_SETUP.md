# Lite3 ROS Integration Setup Guide

## Overview

This guide shows how to integrate the official Deep Robotics `Lite3_ROS` package with the autonomous navigation system.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                  Lite3 Robot (Motion Host)                      │
│              QNX/RK3588 @ 192.168.1.120                         │
└────────────┬──────────────────────────────────┬─────────────────┘
             │ UDP (43897, 43893)               │
             ▼                                  ▼
┌─────────────────────────────────────────────────────────────────┐
│        message_transformer (Perception Host / NX)               │
│                                                                 │
│  Nodes:                                                         │
│  - qnx2ros: Receives robot state via UDP                       │
│    - Publishes: /leg_odom, /imu/data, /joint_states           │
│  - ros2qnx: Sends velocity commands via UDP                    │
│    - Subscribes: /cmd_vel                                      │
│  - nx2app: App communication                                   │
│  - sensor_checker: Monitor sensor health                       │
└────────────┬──────────────────────────────────┬─────────────────┘
             │                                  │
             ▼                                  │
   /leg_odom (nav_msgs/Odometry)              │
   /imu/data (sensor_msgs/Imu)                │
   /joint_states (sensor_msgs/JointState)     │
             │                                  │
             ├──────────────────────────────────┤
             │                                  │
             ▼                                  ▼
┌─────────────────────────────┐    ┌─────────────────────────────┐
│      EGO Planner            │    │   lite3_velocity_controller │
│  - Uses /leg_odom           │    │   - Uses /leg_odom          │
│  - Uses /vk180/pointcloud   │    │   - Subscribes pos_cmd      │
└────────────┬────────────────┘    └────────────┬────────────────┘
             │                                  │
             ▼                                  ▼
    /drone_0_planning/pos_cmd                /cmd_vel
    (quadrotor_msgs/PositionCommand)         (geometry_msgs/Twist)
                                                  │
                                                  ▼
                                          ros2qnx (message_transformer)
                                                  │
                                                  ▼
                                          Lite3 Robot (UDP)
```

## Directory Structure

```
star-nav/
├── Lite3_ROS/                              # Official Deep Robotics package
│   └── message_transformer/
│       └── message_transformer/
│           ├── launch/
│           │   └── message_transformer.launch
│           ├── script/
│           ├── src/
│           ├── msg/
│           └── package.xml
│
└── lite3_ctrl/                             # Our velocity controller
    ├── launch/
    │   └── lite3_autonomous_nav.launch     # Complete system launch
    ├── src/
    │   ├── lite3_velocity_controller.cpp
    │   └── lite3_velocity_controller.py
    ├── config/
    │   └── lite3_ctrl_params.yaml
    └── README.md
```

## Setup Instructions

### 1. Network Configuration

The Lite3 robot's motion host (QNX/RK3588) communicates via UDP. Ensure your perception host (NX) is on the same network:

**Lite3 Motion Host IP:** `192.168.1.120`
**Your Perception Host:** Should be on `192.168.1.x` network

Edit network settings if needed:
```bash
sudo nano /etc/netplan/01-network-manager-all.yaml
```

Example configuration:
```yaml
network:
  version: 2
  ethernets:
    eth0:  # or your interface name
      addresses:
        - 192.168.1.100/24  # Choose an available IP
      gateway4: 192.168.1.1
```

Apply:
```bash
sudo netplan apply
```

Test connection:
```bash
ping 192.168.1.120  # Should reach Lite3 motion host
```

### 2. Build the Packages

```bash
cd ~/src/star-nav

# Build message_transformer
cd Lite3_ROS/message_transformer
catkin_make
source devel/setup.bash

# Go back and build lite3_ctrl
cd ~/src/star-nav
catkin_make --pkg lite3_ctrl
source devel/setup.bash
```

### 3. Configure message_transformer

The default configuration in `message_transformer.launch` should work, but verify:

```xml
<node name="ros2qnx" pkg="message_transformer" type="ros2qnx">
    <param name="remote_ip" value="192.168.1.120"/>  <!-- Lite3 motion host IP -->
    <param name="remote_port" value="43893"/>        <!-- Lite3 UDP port -->
    <param name="local_port" value="43894"/>         <!-- Your listening port -->
</node>
```

## Usage

### Launch the Complete System

```bash
# Make sure robot is powered on and connected to network
roslaunch lite3_ctrl lite3_autonomous_nav.launch use_rviz:=true
```

This will start:
1. **VK180 planner bridge** - Point cloud and vision
2. **message_transformer** - Lite3 robot communication
3. **lite3_velocity_controller** - Trajectory to velocity conversion
4. **EGO planner** - Path planning
5. **Trajectory server** - Trajectory generation
6. **RViz** - Visualization

### Verify Topics

In a new terminal:

```bash
# Check if leg odometry is being published
rostopic hz /leg_odom
# Should see ~100 Hz

# Check IMU data
rostopic hz /imu/data
# Should see ~200 Hz

# Check joint states
rostopic hz /joint_states
# Should see ~100 Hz

# Check velocity commands being sent
rostopic echo /cmd_vel
```

### Switch Robot to Auto Mode

**IMPORTANT:** Before the robot will respond to velocity commands:

1. Use the Lite3 mobile app
2. Switch the robot from **Manual Mode** to **Auto Mode**
3. Keep emergency stop accessible

### Send Test Command

Test if robot responds:

```bash
# Send a simple forward command (0.2 m/s forward)
rostopic pub /cmd_vel geometry_msgs/Twist -r 10 "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```

Robot should start walking forward slowly.

**Press Ctrl+C to stop the command**

### Use 2D Nav Goal

1. Open RViz (should launch automatically)
2. Use "2D Nav Goal" tool in toolbar
3. Click and drag to set destination
4. Robot will plan path and navigate autonomously

## Topic Remapping

The integration uses the following topic connections:

| Source | Topic | Message Type | Consumer |
|--------|-------|--------------|----------|
| message_transformer | /leg_odom | nav_msgs/Odometry | EGO planner, lite3_ctrl |
| message_transformer | /imu/data | sensor_msgs/Imu | (optional) |
| VK180 | /vk180/pointcloud | sensor_msgs/PointCloud2 | EGO planner |
| EGO planner | /drone_0_planning/pos_cmd | quadrotor_msgs/PositionCommand | lite3_ctrl |
| lite3_ctrl | /cmd_vel | geometry_msgs/Twist | message_transformer |

## Troubleshooting

### No leg odometry (`/leg_odom` not publishing)

**Symptoms:**
```bash
rostopic hz /leg_odom
# Shows no data
```

**Solutions:**
1. Check network connection:
   ```bash
   ping 192.168.1.120
   ```
2. Check if robot is powered on
3. Verify UDP ports are not blocked:
   ```bash
   sudo netstat -ulnp | grep 43897
   ```
4. Check message_transformer logs:
   ```bash
   rosnode info /qnx2ros
   ```

### Robot doesn't respond to /cmd_vel

**Symptoms:** Commands published but robot doesn't move

**Solutions:**
1. **Check robot mode:** Must be in Auto Mode (use app)
2. Check if ros2qnx is running:
   ```bash
   rosnode list | grep ros2qnx
   ```
3. Verify velocity commands are being published:
   ```bash
   rostopic echo /cmd_vel
   ```
4. Check ros2qnx logs for UDP send errors
5. Verify robot IP is reachable:
   ```bash
   ping 192.168.1.120
   ```

### Controller timeout warnings

**Symptoms:**
```
[WARN] Odometry timeout - stopping robot
```

**Solutions:**
1. Ensure /leg_odom is publishing reliably:
   ```bash
   rostopic hz /leg_odom
   # Should be ~100 Hz
   ```
2. Check for network latency
3. Adjust timeout in launch file if needed:
   ```xml
   <param name="odom_timeout" value="1.0" />  <!-- Increase from 0.5 -->
   ```

### EGO planner not generating trajectories

**Symptoms:** No /drone_0_planning/pos_cmd messages

**Solutions:**
1. Check if point cloud is available:
   ```bash
   rostopic hz /vk180/pointcloud
   ```
2. Verify odometry is publishing:
   ```bash
   rostopic hz /leg_odom
   ```
3. Try setting a 2D Nav Goal in RViz
4. Check planner logs for errors

## Safety Considerations

### Before Testing
1. ✅ Clear testing area of obstacles and people
2. ✅ Keep emergency stop accessible (app or physical button)
3. ✅ Start with low velocities (0.2-0.5 m/s)
4. ✅ Test manual mode first to verify robot health
5. ✅ Have spotter ready to hit emergency stop

### During Operation
1. ⚠️ Monitor velocity commands: `rostopic echo /cmd_vel`
2. ⚠️ Watch for controller timeout warnings
3. ⚠️ Keep hand near emergency stop
4. ⚠️ Switch to manual mode if unexpected behavior

### Velocity Limits
The controller has safety limits configured:
- `max_linear_vel: 1.0 m/s` (can go up to ~1.5 m/s)
- `max_angular_vel: 1.0 rad/s`

Adjust in `lite3_ctrl/config/lite3_ctrl_params.yaml` or launch file arguments.

## Parameter Tuning

### In Launch File
```xml
<!-- Planner velocity limits -->
<arg name="max_vel" default="0.8" />   <!-- Planner max velocity -->
<arg name="max_acc" default="1.5" />   <!-- Planner max acceleration -->

<!-- Controller parameters -->
<arg name="max_linear_vel" default="1.0" />    <!-- Safety limit -->
<arg name="position_gain" default="0.5" />     <!-- Tracking gain -->
<arg name="velocity_alpha" default="0.3" />    <!-- Smoothing -->
```

### Tuning Guide

**For more aggressive tracking:**
- Increase `position_gain` to 0.7-1.0
- Decrease `velocity_alpha` to 0.1-0.2

**For smoother motion:**
- Decrease `position_gain` to 0.3-0.4
- Increase `velocity_alpha` to 0.4-0.6

**For faster overall speed:**
- Increase `max_vel` (planner)
- Increase `max_linear_vel` (controller)
- Ensure controller limit > planner limit

## Advanced Configuration

### Using Different Odometry Sources

By default, the system uses `/leg_odom` from Lite3. You can also fuse with VK180 VIO:

```xml
<!-- Option 1: Lite3 leg odometry only (default) -->
<arg name="odometry_topic" value="/leg_odom"/>

<!-- Option 2: VK180 VIO only -->
<arg name="odometry_topic" value="/vk180/odom_raw"/>

<!-- Option 3: robot_localization fusion (requires setup) -->
<arg name="odometry_topic" value="/odometry/filtered"/>
```

### Adding IMU to EGO Planner

If you want to use IMU data for better state estimation:

1. Modify launch file to enable IMU topic
2. Configure EGO planner to use IMU (check planner documentation)

## Next Steps

1. ✅ Test basic velocity commands
2. ✅ Verify odometry and point cloud data
3. ⏳ Tune controller parameters
4. ⏳ Test waypoint navigation
5. ⏳ Test 2D Nav Goal navigation
6. ⏳ Add safety zones / virtual boundaries
7. ⏳ Integrate obstacle avoidance enhancements

## References

- [Lite3 ROS Official Repo](https://github.com/DeepRoboticsLab/Lite3_ROS)
- [EGO Planner Paper](https://arxiv.org/abs/2008.08835)
- [VK180 Documentation](https://www.xilinx.com/products/som/kria/vk180.html)

## Support

For issues:
1. Check message_transformer logs: `rosnode info /qnx2ros`
2. Check controller logs: `rosnode info /lite3_velocity_controller`
3. Verify network connectivity to robot
4. Ensure robot is in Auto Mode
