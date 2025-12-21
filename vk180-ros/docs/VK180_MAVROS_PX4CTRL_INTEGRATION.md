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
- **Function**: Converts VK180 eCAL VIO odometry to ROS and provides dual sensor modalities for planning
- **Published Topics**:
  - `/vk180/odom_raw` - Raw VIO odometry before PX4 EKF2 fusion (30 Hz)
  - `/mavros/vision_pose/pose` - Vision pose to PX4 EKF2 for sensor fusion
  - `/vk180/pointcloud` - Combined stereo point cloud in world frame (pcl::PointXYZ)
  - `/vk180/depth/image_rect_raw` - Virtual depth image (16UC1, millimeters, 640×480)
  - `/vk180/depth/camera_info` - Virtual camera intrinsics for depth processing
  - `/vk180/camera_extrinsic` - Camera-to-body transform (nav_msgs::Odometry, latched)
  - `/vk180/image_left` - Left camera RGB (optional)
  - `/vk180/image_right` - Right camera RGB (optional)
  
**Sensor Data Modes**:
- **Point Cloud Mode**: World-frame 3D points for direct obstacle inflation
- **Depth Image Mode**: Virtual body-centered camera for traditional depth-based mapping
- Both modes can be used simultaneously or individually

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
  - `/mavros/local_position/odom` - PX4 fused odometry (VIO + IMU) for localization
  - `/vk180/pointcloud` - Point cloud for obstacle detection (world frame)
  - `/vk180/depth/image_rect_raw` - Virtual depth image (alternative to point cloud)
  - `/vk180/camera_extrinsic` - Camera-to-body transform (remapped from /vins_fusion/extrinsic)
- **Published Topics**:
  - `/drone_0_planning/pos_cmd` - Position commands (quadrotor_msgs::PositionCommand)
  - `/drone_0_planning/bspline` - Trajectory as B-spline
  
**Mapping Modes**:
- **Point Cloud**: Direct world-frame obstacle inflation (recommended)
- **Depth Image**: Traditional raycasting with camera pose transform (optional)

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
2. **VK180 Depth Processing** generates both:
   - **Point cloud** from dual stereo cameras (transformed to world frame)
   - **Virtual depth image** from body-frame point cloud projection
3. **vk180_planner_bridge** publishes:
   - `/vk180/odom_raw` - Raw VIO odometry
   - `/mavros/vision_pose/pose` - Vision pose to PX4 EKF2
   - `/vk180/pointcloud` - World-frame 3D obstacles
   - `/vk180/depth/image_rect_raw` - Virtual depth image (16UC1, millimeters)
   - `/vk180/camera_extrinsic` - Camera-to-body transform
4. **MAVROS & PX4 EKF2** fuses VIO with IMU:
   - `/mavros/local_position/odom` - High-rate fused odometry for control
5. **EGO Planner** subscribes to:
   - `/mavros/local_position/odom` - Fused odometry for state estimation
   - `/vk180/pointcloud` - 3D obstacles for occupancy grid (primary mode)
   - `/vk180/depth/image_rect_raw` - Optional depth-based mapping
   - `/vk180/camera_extrinsic` - Camera-to-body transform for depth processing
   - Builds occupancy grid map
   - Plans collision-free trajectories
   - Publishes position commands to `/drone_0_planning/pos_cmd`
6. **px4ctrl** subscribes to:
   - `/mavros/local_position/odom` - Current state (fused VIO+IMU)
   - `/drone_0_planning/pos_cmd` - Desired trajectory
   - Computes attitude setpoints using geometric control
   - Publishes to `/mavros/setpoint_raw/attitude`
7. **MAVROS** sends MAVLink messages to PX4 flight controller
8. **PX4** executes attitude control and motor mixing

## Topic Remapping

The integration uses topic remapping to connect components:

```xml
<!-- VK180 publishes -->
/vk180/odom_raw → Raw VIO before PX4 fusion
/mavros/vision_pose/pose → Vision input to PX4 EKF2
/vk180/pointcloud → World-frame obstacles for EGO planner
/vk180/depth/image_rect_raw → Virtual depth image
/vk180/camera_extrinsic → Camera-to-body transform

<!-- MAVROS publishes (after PX4 fusion) -->
/mavros/local_position/odom → Fused odometry (VIO + IMU) for control and planning

<!-- EGO planner subscribes and publishes -->
~odom_world ← /mavros/local_position/odom (fused odometry)
~grid_map/odom ← /mavros/local_position/odom (for mapping)
~grid_map/cloud ← /vk180/pointcloud (point cloud obstacles)
~grid_map/depth ← /vk180/depth/image_rect_raw (optional depth)
/vins_fusion/extrinsic ← /vk180/camera_extrinsic (cam2body transform)
~planning/pos_cmd → /drone_0_planning/pos_cmd

<!-- px4ctrl subscribes -->
~odom ← /mavros/local_position/odom (fused odometry)
~cmd ← /drone_0_planning/pos_cmd
```

**Key Data Flow**:
1. VK180 raw VIO → PX4 EKF2 fusion → MAVROS fused odometry
2. Both px4ctrl and EGO planner use the fused odometry for better performance
3. Point cloud transformed to world frame using fused odometry pose

## Coordinate Frames and Transforms

### Frame Definitions

**World Frame** (`world`):
- Fixed inertial frame
- Origin at system initialization point
- Typically ENU (East-North-Up) convention

**Body Frame** (`base_link`):
- FLU (Forward-Left-Up) convention
- X-axis: Forward (drone heading)
- Y-axis: Left (perpendicular to heading)
- Z-axis: Up (opposite to gravity)
- Origin at drone center of mass

**Virtual Camera Frame** (`camera_link`):
- Standard camera convention
- X-axis: Right
- Y-axis: Down
- Z-axis: Forward (optical axis, depth direction)
- Origin at body center (co-located with body frame)

### Camera-to-Body Transform

The virtual camera is body-centered but rotated to follow camera conventions:

**Rotation Matrix (camera → body)**:
```
R_cam2body = [0   0   1]   // body_x = camera_z (forward)
             [1   0   0]   // body_y = camera_x (left = cam right)
             [0  -1   0]   // body_z = -camera_y (up = -cam down)
```

**Published as**:
- Topic: `/vk180/camera_extrinsic`
- Type: `nav_msgs::Odometry`
- Translation: [0, 0, 0] (camera at body center)
- Rotation: Quaternion equivalent of above matrix
- Latched: Yes (published once on startup)

**Usage**:
- EGO planner reads this to transform depth image pixels to world frame
- Transform chain: pixel → camera frame → body frame → world frame
- Required for depth-based occupancy mapping (optional mode)

### Transform Tree

```
world
  └─ odom (identity, static)
      └─ base_link (dynamic, from MAVROS fused odometry)
          └─ camera_link (static, from VK180 extrinsics)
              └─ camera_optical_frame (static, optional)
```

**Publishers**:
- `world → odom`: Static TF publisher (identity transform)
- `odom → base_link`: vk180_planner_bridge (from VK180 raw VIO)
- `base_link → camera_link`: vk180_planner_bridge (from VK180 extrinsics)

**Note**: While `odom → base_link` is published by vk180_planner_bridge for TF tree completeness, control and planning use `/mavros/local_position/odom` which provides the fused pose after PX4 EKF2 sensor fusion.

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
- `use_body_frame_transform`: Enable depth transformation (default: true)
- `subscribe_to_image_stream`: Enable RGB image streaming (default: true)
- `max_depth`: Maximum depth range in meters (default: 8.0)
- `min_depth`: Minimum depth range in meters (default: 0.3)

**Virtual Camera Parameters** (hardcoded in vk180_planner_bridge):
- Resolution: 640×480 pixels
- Focal lengths: fx = fy = 320.0 pixels
- Principal point: cx = 320.0, cy = 240.0 pixels
- Camera position: Body center (0, 0, 0)
- Camera orientation: See "Coordinate Frames and Transforms" section

**Point Cloud Processing**:
- Distance filter: Removes points closer than 0.4m from drone center
- FOV filter: 40° left, 50° right, 10° up, 60° down
- Frame: World frame (transformed using MAVROS fused odometry)

**Depth Image Generation**:
- Encoding: 16UC1 (uint16, millimeters)
- Projection: Virtual body-centered camera with pinhole model
- Z-buffering: Keeps closest depth per pixel
- Frame: camera_link

### EGO Planner Parameters
Set in launch file:
- `map_size_x/y/z`: Planning map dimensions in meters
- `max_vel`: Maximum velocity limit in m/s
- `max_acc`: Maximum acceleration limit in m/s²
- `flight_type`: 1=Interactive 2D Nav Goal, 2=Predefined waypoints
- `planning_horizon`: Planning time horizon in seconds

**Camera Intrinsics** (for depth processing):
- `cx`, `cy`: Principal point (default: 320.0, 240.0)
- `fx`, `fy`: Focal lengths (default: 320.0, 320.0)
- Must match virtual camera parameters in vk180_planner_bridge

**Grid Map Settings**:
- Resolution: 0.15m per voxel
- Depth scaling factor: 1000.0 (converts mm to m)
- Pose type: 2 (uses odometry instead of pose stamped)
- Frame: world

## Testing and Validation

### 1. Check VK180 Odometry
```bash
# Raw VIO odometry
rostopic echo /vk180/odom_raw
rostopic hz /vk180/odom_raw  # Should be ~30 Hz

# Vision pose to PX4
rostopic echo /mavros/vision_pose/pose

# Fused odometry from PX4 EKF2
rostopic echo /mavros/local_position/odom
rostopic hz /mavros/local_position/odom  # Should be high rate (>50 Hz)
```

### 2. Check Sensor Data
```bash
# Point cloud
rostopic echo /vk180/pointcloud
rostopic hz /vk180/pointcloud

# Virtual depth image
rostopic echo /vk180/depth/image_rect_raw
rostopic hz /vk180/depth/image_rect_raw

# Camera info and extrinsics
rostopic echo /vk180/depth/camera_info -n1
rostopic echo /vk180/camera_extrinsic -n1
```

### 3. Check MAVROS Connection
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

### 6. Verify Camera Extrinsics
```bash
# Check camera-to-body transform
rostopic echo /vk180/camera_extrinsic -n1

# Should show:
# - position: [0, 0, 0] (camera at body center)
# - orientation: quaternion for camera→body rotation
# - frame_id: camera_link, child_frame_id: base_link
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
