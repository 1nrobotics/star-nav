# VK180 Autonomous Flight System

This document describes the complete autonomous flight integration with VK180 visual-inertial odometry (VIO), EGO planner, px4ctrl, and MAVROS/PX4.

## System Architecture

```
VK180 Hardware (eCAL)
    ↓
vk180_planner_bridge (ROS)
    ↓ /vk180/odom (Odometry)
    ↓ /vk180/pointcloud (Obstacles)
EGO Planner
    ↓ /drone_0_planning/pos_cmd (Position Command)
px4ctrl (PX4 Controller)
    ↓ /mavros/setpoint_raw/attitude
MAVROS
    ↓ Serial/UDP
PX4 Flight Controller
```

## Components

### 1. VK180 Planner Bridge
- **Package**: `vk180_ros`
- **Node**: `vk180_planner_bridge`
- **Function**: Converts VK180 eCAL VIO odometry to ROS nav_msgs::Odometry
- **Published Topics**:
  - `/vk180/odom` - Odometry from VK180 VIO (30 Hz)
  - `/vk180/depth` - Depth image for obstacle avoidance
  - `/vk180/pointcloud` - Combined stereo point cloud
  - `/vk180/disparity` - Disparity image
  - `/vk180/image_left` - Left camera RGB
  - `/vk180/image_right` - Right camera RGB

### 2. PX4 Controller (px4ctrl)
- **Package**: `px4ctrl`
- **Node**: `px4ctrl_node`
- **Function**: Converts position commands to attitude setpoints for PX4
- **Subscribed Topics**:
  - `~odom` (remapped to `/vk180/odom`) - Odometry input
  - `~cmd` (remapped to `/drone_0_planning/pos_cmd`) - Position commands from EGO planner
- **Published Topics**:
  - `/mavros/setpoint_raw/attitude` - Attitude setpoints to MAVROS

### 3. EGO Planner
- **Package**: `ego_planner`
- **Nodes**: `ego_planner_node`, `traj_server`
- **Function**: Autonomous trajectory planning with obstacle avoidance
- **Subscribed Topics**:
  - `/vk180/odom` - Odometry for localization
  - `/vk180/pointcloud` - Point cloud for obstacle detection
- **Published Topics**:
  - `/drone_0_planning/pos_cmd` - Position commands (quadrotor_msgs::PositionCommand)
  - `/drone_0_planning/bspline` - Trajectory as B-spline

### 4. MAVROS
- **Package**: `mavros`
- **Function**: MAVLink communication bridge between ROS and PX4
- **Connection**:
  - **Simulation**: UDP `udp://:14540@127.0.0.1:14557`
  - **Real Hardware**: Serial `/dev/ttymxc2:921600`

## Launch Files

### Main Integration Launch
```bash
roslaunch vk180_ros vk180_autonomous_flight.launch
```

**Arguments**:
- `use_sim:=true/false` - Use simulation or real hardware (default: false)
- `use_body_frame_transform:=true/false` - Transform depth to body frame (default: true)
- `use_rviz:=true/false` - Launch RViz visualization (default: true)
- `drone_id` - Drone ID for multi-robot scenarios (default: 0)
- `map_size_x` - Map size in X direction in meters (default: 50)
- `map_size_y` - Map size in Y direction in meters (default: 25)
- `map_size_z` - Map size in Z direction in meters (default: 3.0)
- `flight_type` - 1=2D Nav Goal via RViz, 2=Waypoint following (default: 1)
- `max_vel` - Maximum velocity in m/s (default: 1.0)
- `max_acc` - Maximum acceleration in m/s² (default: 2.0)
- `fcu_url_sim` - FCU URL for simulation (default: udp://:14540@127.0.0.1:14557)
- `fcu_url_real` - FCU URL for real hardware (default: /dev/ttymxc2:921600)

### Example Usage

**Real Hardware**:
```bash
roslaunch vk180_ros vk180_autonomous_flight.launch use_sim:=false
```

**Simulation**:
```bash
roslaunch vk180_ros vk180_autonomous_flight.launch use_sim:=true
```

**With Custom Flight Parameters**:
```bash
roslaunch vk180_ros vk180_autonomous_flight.launch \
    use_sim:=false \
    max_vel:=1.5 \
    max_acc:=3.0 \
    flight_type:=1
```

**Navigation via RViz**:
1. Launch the system:
   ```bash
   roslaunch vk180_ros vk180_autonomous_flight.launch
   ```
2. In RViz, use the "2D Nav Goal" tool to set waypoints
3. EGO planner will generate collision-free trajectories
4. px4ctrl tracks the trajectory and controls the drone

## Data Flow

1. **VK180 VIO** generates odometry at ~30 Hz from stereo cameras and IMU
2. **VK180 Depth Processing** generates point cloud from stereo disparity
3. **vk180_planner_bridge** publishes:
   - `/vk180/odom` - Odometry for localization
   - `/vk180/pointcloud` - 3D obstacles for planning
4. **EGO Planner** subscribes to odometry and point cloud:
   - Builds occupancy grid map from point cloud
   - Uses odometry for state estimation
   - Plans collision-free trajectories
   - Publishes position commands to `/drone_0_planning/pos_cmd`
5. **px4ctrl** subscribes to:
   - `/vk180/odom` - Current state
   - `/drone_0_planning/pos_cmd` - Desired trajectory
   - Computes attitude setpoints using geometric control
   - Publishes to `/mavros/setpoint_raw/attitude`
6. **MAVROS** sends MAVLink messages to PX4 flight controller
7. **PX4** executes attitude control and motor mixing

## Topic Remapping

The integration uses topic remapping to connect components:

```xml
<!-- VK180 publishes -->
/vk180/odom → odometry for both EGO planner and px4ctrl
/vk180/pointcloud → obstacles for EGO planner

<!-- EGO planner subscribes and publishes -->
~odom_world ← /vk180/odom
~grid_map/cloud ← /vk180/pointcloud
~planning/pos_cmd → /drone_0_planning/pos_cmd

<!-- px4ctrl subscribes -->
~odom ← /vk180/odom
~cmd ← /drone_0_planning/pos_cmd
```

This allows seamless integration between VK180, EGO planner, and px4ctrl.

## Comparison with shiviz_px4_nav

### shiviz_px4_nav Architecture
```
px4_offboard_node (Waypoint Navigator)
    ↓ /position_cmd
px4ctrl (implied, not in launch file)
    ↓ /mavros/setpoint_raw/attitude
MAVROS
    ↓
PX4
```

### VK180 Integration Architecture
```
VK180 Hardware
    ↓
vk180_planner_bridge
    ↓ /vk180/odom
    ↓ /vk180/pointcloud
EGO Planner (Autonomous Path Planning)
    ↓ /drone_0_planning/pos_cmd
px4ctrl
    ↓ /mavros/setpoint_raw/attitude
MAVROS
    ↓
PX4
```

**Key Features**:
1. **Odometry Source**: VK180 VIO instead of external mocap/VINS
2. **Sensor Data**: VK180 provides stereo depth and point clouds
3. **Autonomous Planning**: EGO planner for obstacle avoidance
4. **Complete Integration**: Single launch file for entire system
5. **Interactive Control**: Set goals via RViz 2D Nav Goal tool

## Configuration

### PX4 Controller Parameters
Located in `px4ctrl/config/ctrl_param_fpv.yaml`:
- Gains (Kp, Kv, Ki for position/velocity)
- Mass and inertia parameters
- Maximum tilt angles
- Thrust limits

### VK180 Parameters
Set in launch file:
- `use_body_frame_transform`: Enable depth transformation
- `max_depth`: Maximum depth range (default: 8.0 m)
- `min_depth`: Minimum depth range (default: 0.3 m)
- `subscribe_to_image_stream`: Enable RGB image streaming

### EGO Planner Parameters
Set in launch file:
- `map_size_x/y/z`: Planning map dimensions
- `max_vel`: Maximum velocity limit
- `max_acc`: Maximum acceleration limit
- `flight_type`: 1=Interactive 2D Nav Goal, 2=Predefined waypoints
- `planning_horizon`: Planning time horizon in seconds

## Testing and Validation

### 1. Check VK180 Odometry
```bash
rostopic echo /vk180/odom
rostopic hz /vk180/odom  # Should be ~30 Hz
```

### 2. Check MAVROS Connection
```bash
rostopic echo /mavros/state
# armed: false
# connected: true
# guided: false
# mode: ""
```

### 3. Check px4ctrl Output
```bash
rostopic echo /mavros/setpoint_raw/attitude
rostopic echo /drone_0_planning/pos_cmd
```

### 4. Check EGO Planner
```bash
# Check planning output
rostopic echo /drone_0_planning/data_display

# Check trajectory
rostopic echo /drone_0_planning/bspline
```

### 5. Monitor TF Tree
```bash
rosrun rqt_tf_tree rqt_tf_tree
# Should show: world -> odom -> base_link -> camera_link
```

## Integration with EGO Planner

EGO planner is **already integrated** in the main launch file. The complete autonomous navigation system works as follows:

1. **Launch Complete System**:
   ```bash
   roslaunch vk180_ros vk180_autonomous_flight.launch
   ```

2. **Set Navigation Goals**:
   - **Option 1 (Interactive)**: Use RViz "2D Nav Goal" tool to set waypoints
   - **Option 2 (Predefined)**: Set `flight_type:=2` and define waypoints in launch file

3. **System Behavior**:
   - VK180 provides odometry and obstacle detection
   - EGO planner plans collision-free trajectories
   - px4ctrl tracks the planned trajectory
   - PX4 executes motor commands

### Advanced Configuration

Modify planning behavior by changing launch arguments:

```bash
roslaunch vk180_ros vk180_autonomous_flight.launch \
    flight_type:=2 \
    max_vel:=2.0 \
    max_acc:=4.0 \
    map_size_x:=100 \
    map_size_y:=100
```

## Troubleshooting

### No Odometry Published
- Check VK180 hardware connection
- Verify eCAL daemon is running: `ecal_info`
- Check vk180_planner_bridge logs

### MAVROS Not Connected
- Check serial port permissions: `ls -l /dev/ttymxc2`
- Verify PX4 is running and accessible
- Check MAVROS logs for connection errors

### px4ctrl Not Responding
- Verify odometry topic: `rostopic echo /vk180/odom`
- Check parameter file loaded correctly
- Monitor `/debugPx4ctrl` topic for debug info

### Poor Flight Performance
- Tune px4ctrl gains in `ctrl_param_fpv.yaml`
- Verify odometry quality (low drift, smooth)
- Check for delays in odometry pipeline

## Future Enhancements

1. **Visual Odometry Fusion**: Combine VK180 VIO with PX4 EKF2
2. **Failsafe Integration**: Use PX4 GPS as backup when VIO fails
3. **Depth-based Collision Avoidance**: Integrate VK180 depth with px4ctrl
4. **Multi-sensor Fusion**: Combine VK180 with other sensors
5. **Automatic Calibration**: Auto-calibrate VK180 to body frame

## References

- VK180 SDK Documentation: `/opt/vilota/`
- PX4 Documentation: https://docs.px4.io/
- MAVROS: http://wiki.ros.org/mavros
- px4ctrl: Custom controller for position tracking
