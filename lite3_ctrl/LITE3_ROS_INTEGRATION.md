# Lite3_ROS Integration Guide

## Overview

This document explains how the Lite3_ROS package (Deep Robotics official ROS bridge) is integrated with the autonomous navigation system.

## Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                    Lite3 Robot Hardware                        │
│                   (QNX Real-time System)                       │
└──────────────────────┬─────────────────────────────────────────┘
                       │ UDP (192.168.1.120:43893)
                       ▼
┌────────────────────────────────────────────────────────────────┐
│              message_transformer Package                       │
│              (Lite3_ROS official bridge)                       │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│  qnx2ros (server_port: 43897)                                 │
│    ├─► /leg_odom (PoseWithCovarianceStamped)                  │
│    ├─► /leg_odom2 (nav_msgs/Odometry) ◄─ Used for control!   │
│    ├─► /joint_states (sensor_msgs/JointState)                 │
│    ├─► /imu/data (sensor_msgs/Imu)                            │
│    └─► /handle_state (geometry_msgs/Twist)                    │
│                                                                │
│  ros2qnx (remote_ip: 192.168.1.120, remote_port: 43893)       │
│    └─► Subscribes to /cmd_vel ◄─ Commands to robot!          │
│                                                                │
│  nx2app (server_port: 43899)                                  │
│    └─► App-level communication                                │
│                                                                │
│  sensor_checker                                                │
│    └─► Monitors all sensor topics for health                  │
│                                                                │
└────────────────────────────────────────────────────────────────┘
                       │
                       │ ROS Topics
                       │
        ┌──────────────┼──────────────────┐
        │              │                   │
        ▼              ▼                   ▼
┌────────────┐  ┌──────────────┐  ┌──────────────────┐
│  VK180     │  │ EGO Planner  │  │ Lite3 Velocity   │
│  Sensors   │  │              │  │ Controller       │
│            │  │ Input:       │  │                  │
│ Output:    │  │ - /leg_odom2 │  │ Input:           │
│ - Point    │  │ - /vk180/    │  │ - /leg_odom2     │
│   Cloud    │  │   pointcloud │  │ - trajectory cmd │
└────────────┘  │              │  │                  │
                │ Output:      │  │ Output:          │
                │ - trajectory │  │ - /cmd_vel       │
                └──────────────┘  └──────────────────┘
```

## Package Structure

### Lite3_ROS/message_transformer

This is the official Deep Robotics ROS bridge package.

**Location:** `/home/yingcai/src/star-nav/Lite3_ROS/message_transformer/message_transformer/`

**Key Nodes:**

1. **qnx2ros** - Receives data from robot
   - Server port: 43897
   - Publishes robot state to ROS topics
   - Odometry format: Both PoseWithCovarianceStamped and nav_msgs/Odometry

2. **ros2qnx** - Sends commands to robot
   - Local port: 43894
   - Remote port: 43893
   - Remote IP: 192.168.1.120 (Lite3 robot)
   - Subscribes to `/cmd_vel` (geometry_msgs/Twist)
   - Converts velocity commands to robot protocol

3. **nx2app** - App-level communication
   - Server port: 43899
   - Additional communication channel

4. **sensor_checker** - Health monitoring
   - Checks all sensor topics for data flow
   - Configurable timeout limits

## Topics Published by Lite3_ROS

### Primary Odometry (USED)
- **Topic:** `/leg_odom2`
- **Type:** `nav_msgs/Odometry`
- **Frame:** `odom`
- **Usage:** EGO planner and velocity controller subscribe to this
- **Content:** 
  - Position (x, y, z)
  - Orientation (quaternion)
  - Linear velocity
  - Angular velocity

### Alternative Odometry
- **Topic:** `/leg_odom`
- **Type:** `geometry_msgs/PoseWithCovarianceStamped`
- **Note:** Same data, different message type

### IMU Data
- **Topic:** `/imu/data`
- **Type:** `sensor_msgs/Imu`
- **Rate:** 200 Hz
- **Usage:** Can be fused with odometry if needed

### Joint States
- **Topic:** `/joint_states`
- **Type:** `sensor_msgs/JointState`
- **Content:** All 12 leg joint positions/velocities

### Handle State
- **Topic:** `/handle_state`
- **Type:** `geometry_msgs/Twist`
- **Content:** Physical handle input from robot

## Topics Subscribed by Lite3_ROS

### Velocity Commands (USED)
- **Topic:** `/cmd_vel`
- **Type:** `geometry_msgs/Twist`
- **Usage:** Our velocity controller publishes here
- **Format:**
  ```
  linear.x: forward velocity (m/s)
  linear.y: lateral velocity (m/s)
  linear.z: not used (robot handles height)
  angular.z: yaw rate (rad/s)
  ```

### Corrected Velocity Commands
- **Topic:** `/cmd_vel_corrected`
- **Type:** `geometry_msgs/Twist`
- **Note:** Alternative input topic, same function

## Integration in Launch File

The `lite3_autonomous_nav.launch` includes message_transformer:

```xml
<!-- Lite3 Message Transformer - Bridge between ROS and Lite3 robot -->
<include file="$(find message_transformer)/launch/message_transformer.launch" />
```

This launches all 4 nodes:
- qnx2ros (publishes /leg_odom2)
- ros2qnx (subscribes to /cmd_vel)
- nx2app
- sensor_checker

## Data Flow

### Navigation Loop (50 Hz)

1. **Lite3 Robot** → UDP → **qnx2ros**
   - Robot computes leg odometry using IMU + joint encoders
   - Publishes to `/leg_odom2` at ~100 Hz

2. **EGO Planner** subscribes to `/leg_odom2`
   - Uses odometry for localization in map
   - Plans collision-free trajectory
   - Publishes trajectory to trajectory server

3. **Lite3 Velocity Controller** subscribes to:
   - `/leg_odom2` for current state
   - `/drone_0_planning/pos_cmd` for desired trajectory
   - Computes velocity command: `v = v_desired + Kp * error`
   - Publishes to `/cmd_vel`

4. **ros2qnx** subscribes to `/cmd_vel`
   - Converts Twist message to robot protocol
   - Sends via UDP to robot at 192.168.1.120:43893

5. **Lite3 Robot** executes velocity command
   - Internal controller achieves desired velocity
   - Updates joint positions
   - Loop repeats

## Network Configuration

The Lite3 robot must be reachable at:
- **IP:** 192.168.1.120
- **Ports:** 43893 (command input), 43897 (state output)

Configure your network interface:
```bash
sudo ifconfig eth0 192.168.1.100 netmask 255.255.255.0
```

Or use NetworkManager for persistent config.

## Testing the Integration

### 1. Check Lite3 Connection
```bash
# Ping the robot
ping 192.168.1.120

# Should respond if robot is powered and connected
```

### 2. Launch message_transformer
```bash
roslaunch message_transformer message_transformer.launch
```

**Expected output:**
- "qnx2ros started"
- "ros2qnx started"
- "Connected to robot at 192.168.1.120:43893"

### 3. Verify Topics
```bash
# Check odometry is publishing
rostopic hz /leg_odom2
# Should show ~100 Hz

# Check odometry data
rostopic echo /leg_odom2 -n 1

# Check IMU
rostopic hz /imu/data
# Should show ~200 Hz

# Check joint states
rostopic hz /joint_states
```

### 4. Test Velocity Command
```bash
# Send a simple forward velocity
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Robot should walk forward slowly
# Ctrl+C to stop
```

### 5. Launch Full Navigation
```bash
roslaunch lite3_ctrl lite3_autonomous_nav.launch use_rviz:=true
```

## Troubleshooting

### No odometry data
- Check network connection: `ping 192.168.1.120`
- Check if qnx2ros is running: `rosnode list | grep qnx2ros`
- Check for errors: `rosnode info /qnx2ros`
- Verify robot is powered on and in ready state

### Robot doesn't respond to commands
- Check ros2qnx is running: `rosnode list | grep ros2qnx`
- Verify /cmd_vel has data: `rostopic echo /cmd_vel`
- Check network connection: `ping 192.168.1.120`
- Ensure robot is not in e-stop or error state
- Check robot mode (may need to be in velocity control mode)

### Sensor checker warnings
```bash
# View sensor checker output
rosnode info /sensor_checker

# Check which sensors are silent
rostopic echo /sensor_checker_status
```

### Odometry drift
- Leg odometry can drift over time (dead reckoning)
- Consider fusing with VK180 VIO for better accuracy
- Use SLAM with VK180 point cloud for long-term stability

## Parameters

All parameters are in `message_transformer.launch`:

```xml
<!-- qnx2ros -->
<param name="server_port" value="43897" />
<param name="is_imu_pub" value="true" />

<!-- ros2qnx -->
<param name="vel_x_factor" value="1.0" />      <!-- Velocity scaling -->
<param name="local_port" value="43894"/>
<param name="remote_port" value="43893"/>
<param name="remote_ip" value="192.168.1.120"/>  <!-- Robot IP -->
<param name="isdebug" value="false"/>

<!-- sensor_checker timeouts (seconds) -->
<param name="imu_silent_limit" value="3"/>
<param name="odom_silent_limit" value="3"/>
<param name="joint_silent_limit" value="3"/>
```

## Safety Notes

1. **Always have e-stop ready** - Physical emergency stop button
2. **Test in open space first** - Ensure clear area around robot
3. **Start with low velocities** - max_vel: 0.5 m/s initially
4. **Monitor odometry quality** - Check for drift or jumps
5. **Watch for timeout warnings** - Sensor checker monitors health

## Building

The Lite3_ROS package is already built. If you need to rebuild:

```bash
cd ~/catkin_ws
catkin_make --pkg message_transformer
source devel/setup.bash
```

## Future Improvements

- [ ] Fuse VK180 VIO with leg odometry for better accuracy
- [ ] Add odometry drift correction using SLAM
- [ ] Implement terrain-aware velocity limits
- [ ] Add battery monitoring from robot state
- [ ] Emergency stop integration
- [ ] Gait mode selection based on terrain
