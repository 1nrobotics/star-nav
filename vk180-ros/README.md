# VK180 ROS Bridge

This package provides a ROS bridge for VK180 eCAL odometry and disparity data, converting eCAL messages to standard ROS topics.

## Features

- **Odometry Bridge**: Converts eCAL `Odometry3d` messages to ROS `nav_msgs/Odometry`
- **Disparity Bridge**: Converts eCAL `Disparity` messages to ROS `sensor_msgs/Image` and `sensor_msgs/CameraInfo`
- **Transform Publishing**: Optionally publishes TF transforms for odometry
- **Configurable Topics**: Support for different eCAL topic names and frame IDs

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/vk180/odometry` | `nav_msgs/Odometry` | VIO odometry from eCAL |
| `/vk180/disparity/image` | `sensor_msgs/Image` | Disparity/depth image from eCAL |
| `/vk180/disparity/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics and stereo info |

## TF Frames

| Frame | Description |
|-------|-------------|
| `odom` | Odometry reference frame |
| `base_link` | Robot base frame |
| `camera_link` | Camera frame |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `odom_frame_id` | string | `"odom"` | Odometry frame ID |
| `base_frame_id` | string | `"base_link"` | Base frame ID |
| `camera_frame_id` | string | `"camera_link"` | Camera frame ID |
| `ecal_odom_topic` | string | `"S1/vio_odom"` | eCAL odometry topic name |
| `ecal_disparity_topic` | string | `"S1/stereo1_l/disparity"` | eCAL disparity topic name |
| `publish_tf` | bool | `true` | Whether to publish TF transforms |

## Usage

### 1. Build the Package

```bash
cd ~/catkin_ws
catkin build vk180_ros
source devel/setup.bash
```

### 2. Launch the Bridge

```bash
# Basic launch
roslaunch vk180_ros ecal_ros_bridge.launch

# With custom parameters
roslaunch vk180_ros ecal_ros_bridge.launch odom_frame_id:=map base_frame_id:=robot_base
```

### 3. Using Configuration File

```bash
# Load parameters from config file
rosparam load $(rospack find vk180_ros)/config/bridge_config.yaml /ecal_ros_bridge/
roslaunch vk180_ros ecal_ros_bridge.launch
```

### 4. Monitor Topics

```bash
# List active topics
rostopic list | grep vk180

# Monitor odometry
rostopic echo /vk180/odometry

# Monitor disparity image info
rostopic echo /vk180/disparity/camera_info

# View disparity image in RViz or image_view
rosrun image_view image_view image:=/vk180/disparity/image
```

## Dependencies

### System Dependencies
- VK180 SDK (vilota)
- eCAL libraries
- CapnProto

### ROS Dependencies
- roscpp
- nav_msgs
- sensor_msgs
- geometry_msgs
- tf2_ros

## Configuration Examples

### Different eCAL Topics
```yaml
ecal_odom_topic: "S0/vio_odom"        # Use sensor S0 instead of S1
ecal_disparity_topic: "S0/stereo1_l/disparity"
```

### Custom Frame IDs
```yaml
odom_frame_id: "map"
base_frame_id: "drone_base"
camera_frame_id: "stereo_camera"
```

### Disable TF Publishing
```yaml
publish_tf: false  # Useful when using robot_localization or other tf publishers
```

## Troubleshooting

### eCAL Connection Issues
1. Ensure eCAL daemon is running: `ecal_daemon`
2. Check available eCAL topics: `ecal_info`
3. Verify VK180 SDK installation: `/opt/vilota/`

### ROS Topics Not Publishing
1. Check node status: `rosnode info /ecal_ros_bridge`
2. Verify eCAL topics are active: `ecal_mon`
3. Check ROS master: `rostopic list`

### Build Issues
1. Ensure VK180 SDK paths are correct in CMakeLists.txt
2. Check CapnProto installation
3. Verify ROS dependencies: `rosdep install --from-paths . --ignore-src -r -y`

## Original Test Executables

The package also builds the original eCAL test executables:
- `receive` - All topic receiver
- `odom` - Odometry-only receiver  
- `disparity` - Disparity-only receiver

```bash
# Run original test programs
rosrun vk180_ros odom
rosrun vk180_ros disparity
rosrun vk180_ros receive
```