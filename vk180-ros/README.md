# VK180 ROS Integration Package

Comprehensive ROS package for integrating VK180 hardware with EGO Planner for autonomous navigation with PX4.

## Features

- **Dual Sensor Modalities**: Point cloud and virtual depth image for flexible obstacle detection
- **VIO-IMU Fusion**: VK180 VIO integrated with PX4 EKF2 for robust state estimation
- **Autonomous Planning**: EGO planner with collision-free trajectory generation
- **Camera-to-Body Extrinsics**: Proper coordinate frame transforms for depth processing
- **Complete Integration**: Single launch file for full autonomous flight system

## Published Topics

### Odometry
| Topic | Type | Description |
|-------|------|-------------|
| `/vk180/odom_raw` | `nav_msgs/Odometry` | Raw VIO odometry (30 Hz) |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | Vision pose to PX4 EKF2 |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | Fused odometry (VIO+IMU, >50 Hz) |

### Sensor Data
| Topic | Type | Description |
|-------|------|-------------|
| `/vk180/pointcloud` | `sensor_msgs/PointCloud2` | World-frame point cloud (pcl::PointXYZ) |
| `/vk180/depth/image_rect_raw` | `sensor_msgs/Image` | Virtual depth image (16UC1, mm, 640×480) |
| `/vk180/depth/camera_info` | `sensor_msgs/CameraInfo` | Virtual camera intrinsics |
| `/vk180/camera_extrinsic` | `nav_msgs/Odometry` | Camera-to-body transform (latched) |
| `/vk180/image_left` | `sensor_msgs/Image` | Left RGB image (optional) |
| `/vk180/image_right` | `sensor_msgs/Image` | Right RGB image (optional) |

### Control & Planning
| Topic | Type | Description |
|-------|------|-------------|
| `/drone_0_planning/pos_cmd` | `quadrotor_msgs::PositionCommand` | EGO planner trajectory |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | px4ctrl output |

## TF Frames

```
world
  └─ odom (static)
      └─ base_link (dynamic, from MAVROS fused odom)
          └─ camera_link (static, from VK180 extrinsics)
```

**Coordinate Conventions**:
- **World**: Fixed inertial frame (ENU)
- **Body** (`base_link`): FLU (Forward-Left-Up)
- **Camera** (`camera_link`): X=right, Y=down, Z=forward (standard camera)## Usage



## 🚀 Quick Start### 1. Build the Package



### 1. Complete System Launch (Recommended)```bash

## Quick Start

### 1. Build Package

```bash
cd ~/catkin_ws
catkin build vk180_ros
source devel/setup.bash
```

### 2. Launch Full Autonomous System

```bash
# Complete system with VIO, PX4, EGO planner
roslaunch vk180_ros vk180_autonomous_flight.launch
```

### 3. Monitor Topics

```bash
# List active topics
rostopic list | grep vk180

# Monitor odometry
rostopic echo /vk180/odom_raw

# Monitor depth image
rostopic echo /vk180/depth/image_rect_raw

# Verify camera extrinsics
rostopic echo /vk180/camera_extrinsic
```

### 4. RViz Visualization

```bash
rviz -d $(rospack find vk180_ros)/config/vk180_planner_visualization.rviz
```

## Virtual Camera Specifications

**Virtual Depth Camera** (body-centered):
- Resolution: 640×480 pixels
- Encoding: 16UC1 (uint16 millimeters)
- Focal length: fx=fy=320.0 pixels
- Principal point: cx=320.0, cy=240.0 pixels
- Camera-to-body rotation: [[0,0,1], [1,0,0], [0,-1,0]]

The virtual depth image represents the environment as seen from the drone body center, with camera orientation aligned to body frame through the published extrinsics.

## Dependencies

### System Dependencies
- VK180 SDK (Vilota)
- eCAL (Eclipse Cyclone CAL)
- CapnProto

### ROS Dependencies
- roscpp
- nav_msgs
- sensor_msgs
- geometry_msgs
- tf2_ros
- quadrotor_msgs
- mavros_msgs

## Configuration

### Launch File Parameters

**vk180_autonomous_flight.launch** includes all necessary parameters:

```xml
<!-- Sensor topics -->
<arg name="cloud_topic" value="/vk180/pointcloud"/>
<arg name="depth_topic" value="/vk180/depth/image_rect_raw"/>
<arg name="pose_topic" value="/mavros/local_position/odom"/>

<!-- Virtual camera intrinsics -->
<arg name="cx" value="320.0"/>
<arg name="cy" value="240.0"/>
<arg name="fx" value="320.0"/>
<arg name="fy" value="320.0"/>

<!-- Camera extrinsics -->
<remap from="/vins_fusion/extrinsic" to="/vk180/camera_extrinsic"/>
```

### Choosing Sensor Modality

**Point Cloud Mode** (default):
- More accurate 3D reconstruction
- Better for sparse obstacles
- Higher computational cost
- Set: `sensor_pose_topic="/vk180/pointcloud"`

**Depth Image Mode**:
- Faster processing
- Better for dense environments
- Requires camera extrinsics
- Set: `depth_topic="/vk180/depth/image_rect_raw"`

Both modes can be used simultaneously for robust obstacle detection.

## Architecture

### System Integration

```
VK180 Hardware
    ├─> eCAL Protocol
    └─> vk180_planner_bridge Node
            ├─> /vk180/odom_raw         → MAVROS vision_pose → PX4 EKF2
            ├─> /vk180/pointcloud       → EGO Planner (occupancy grid)
            ├─> /vk180/depth/image_rect_raw → EGO Planner (grid map)
            ├─> /vk180/camera_extrinsic → EGO Planner (depth transform)
            └─> /vk180/image_left/right → Visualization

PX4 Flight Controller
    └─> MAVROS
            └─> /mavros/local_position/odom → EGO Planner (pose feedback)

EGO Planner
    └─> /drone_0_planning/pos_cmd → px4ctrl → /mavros/setpoint_raw/attitude
```

### Data Flow

1. **VIO-IMU Fusion**: VK180 VIO + PX4 IMU → EKF2 → Fused odometry (>50 Hz)
2. **Obstacle Detection**: Point cloud + depth image → Occupancy grid map
3. **Trajectory Planning**: EGO planner generates collision-free paths
4. **Control**: px4ctrl converts trajectories to attitude setpoints → PX4

## Development

### Building

```bash
cd ~/catkin_ws
catkin build vk180_ros
# or
catkin_make --pkg vk180_ros
```

### Testing Components

```bash
# Run individual test utilities
rosrun vk180_ros sensor_extrinsics    # Check camera calibration
rosrun vk180_ros depth_conversion_test # Validate depth processing
rosrun vk180_ros odom                 # Test VK180 odometry stream
rosrun vk180_ros disparity            # Test VK180 disparity stream
```

### Legacy Scripts

```bash
# TMUX-based launch (legacy, for development)
cd /home/yc/catkin_ws/src/vk180-ros
./scripts/launch_vk180_planner.sh

# Simple testing launch
./scripts/simple_launch.sh
```

## Troubleshooting

### eCAL Connection Issues

1. Ensure eCAL daemon is running: `ecal_daemon`
2. Check available eCAL topics: `ecal_info`
3. Verify VK180 SDK installation: `/opt/vilota/`

### ROS Topics Not Publishing

1. Check node status: `rosnode info /vk180_planner_bridge`
2. Verify eCAL topics are active: `ecal_mon`
3. Check ROS master: `rostopic list`
4. Verify camera extrinsics: `rostopic echo /vk180/camera_extrinsic`

### Planning Issues

1. **Depth image not updating**: Check `/vk180/depth/image_rect_raw` and camera extrinsics
2. **Point cloud empty**: Verify VK180 disparity stream is active
3. **Poor trajectory quality**: Tune EGO planner parameters in launch file
4. **Coordinate frame errors**: Verify TF tree with `rosrun tf view_frames`

### Build Issues

1. Ensure VK180 SDK paths are correct in `CMakeLists.txt`
2. Check CapnProto installation: `pkg-config --modversion capnp`
3. Verify ROS dependencies: `rosdep install --from-paths . --ignore-src -r -y`

## Performance

- **Odometry Rate**: 30 Hz (VK180 raw), >50 Hz (PX4 fused)
- **Point Cloud Rate**: 10-30 Hz depending on scene
- **Depth Image Rate**: 30 Hz
- **Planning Update**: 10-20 Hz
- **Memory Usage**: Optimized Cap'n Proto zero-copy handling

## Package Structure

```
vk180-ros/
├── src/
│   ├── nodes/
│   │   ├── vk180_planner_bridge.cpp    # Main integration node
│   │   ├── ecal_ros_bridge_node.cpp    # Legacy basic bridge
│   │   └── AllTopicReceiver.cpp        # Development debugging
│   └── utils/
│       ├── odom.cpp                    # Odometry stream testing
│       ├── disparity.cpp               # Disparity stream testing
│       ├── sensor_extrinsics.cpp       # Camera calibration
│       └── depth_conversion_test.cpp   # Depth validation
├── launch/
│   ├── vk180_autonomous_flight.launch  # Complete autonomous system
│   ├── advanced_param_exp.xml          # EGO planner parameters
│   └── ecal_ros_bridge.launch          # Legacy bridge
├── config/
│   └── vk180_planner_visualization.rviz
├── scripts/
│   ├── launch_vk180_planner.sh         # Legacy TMUX launch
│   └── simple_launch.sh                # Legacy simple launch
├── docs/
│   ├── DEPTH_CONVERSION_GUIDE.md
│   └── VK180_MAVROS_PX4CTRL_INTEGRATION.md
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Additional Documentation

- **VK180_MAVROS_PX4CTRL_INTEGRATION.md**: Complete system architecture and integration details
- **DEPTH_CONVERSION_GUIDE.md**: Depth image processing and coordinate transforms
- **scripts/README.md**: Utility scripts documentation

## License

See individual package licenses. VK180 SDK is proprietary (Vilota).