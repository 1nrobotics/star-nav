# VK180 Depth Conversion and Planner Integration

## Overview

This document explains how to extract sensor extrinsics from VK180 stereo cameras and convert depth data from camera frame to body frame for improved EGO planner integration.

## Key Components

### 1. Sensor Extrinsics Extraction (`depth_conversion_test.cpp`)

**Purpose**: Extract camera-to-body transformation matrix from VK180 S1/stereo1_l stream.

**Key Features**:
- Automatically extracts extrinsics from `S1/stereo1_l` image stream
- Provides complete camera-to-body transformation matrix
- Demonstrates depth point conversion from camera to body frame
- Shows integration examples for planner usage

**Usage**:
```bash
cd /home/yc/catkin_ws
catkin_make
./devel/lib/vk180_ros/depth_conversion_test
```

**Output**:
- Camera-to-body transformation matrix
- Example depth point conversions
- Integration guidance for planner

### 2. VK180 Planner Bridge (`vk180_planner_bridge.cpp`)

**Purpose**: Complete VK180-to-ROS bridge with virtual body-centered camera for FUEL planner.

**Key Features**:
- Virtual camera located at body origin (no camera extrinsics needed)
- Body-frame point cloud converted to depth via ideal pinhole model
- Compile-time camera intrinsics constants (fx=fy=cx=320, cy=240)
- Direct vision pose publishing to PX4 EKF2
- World-frame point cloud for mapping

**Topics Published**:
- `/vk180/odom_raw` - Raw VIO odometry (nav_msgs::Odometry)
- `/mavros/vision_pose/pose` - Vision pose to PX4 EKF2 (geometry_msgs::PoseStamped)
- `/vk180/depth/image_rect_raw` - Body frame depth (sensor_msgs::Image, 16UC1)
- `/vk180/depth/camera_info` - Virtual camera intrinsics (sensor_msgs::CameraInfo)
- `/vk180/pointcloud` - World-frame point cloud (sensor_msgs::PointCloud2)
- `/vk180/image_left` - Left camera RGB image (sensor_msgs::Image)
- `/vk180/image_right` - Right camera RGB image (sensor_msgs::Image)

**Deprecated Topics** (removed in latest version):
- `/vk180/depth` - No longer published
- `/vk180/disparity` - No longer published
- `/vk180/camera_pose` - Use `/mavros/vision_pose/pose` directly

## Coordinate Frame Transformation

### Virtual Camera at Body Origin

The current implementation uses a **virtual camera centered at the body frame origin** to simplify depth processing:

```cpp
// Virtual camera transform: cam_x=body_y, cam_y=-body_z, cam_z=body_x
float cam_x = body_y;
float cam_y = -body_z;
float cam_z = body_x;

// Project to image using compile-time constants
const float VIRTUAL_CAM_FX = 320.0f;
const float VIRTUAL_CAM_FY = 320.0f;
const float VIRTUAL_CAM_CX = 320.0f;
const float VIRTUAL_CAM_CY = 240.0f;

int u = static_cast<int>(VIRTUAL_CAM_FX * cam_x / cam_z + VIRTUAL_CAM_CX);
int v = static_cast<int>(VIRTUAL_CAM_FY * cam_y / cam_z + VIRTUAL_CAM_CY);
uint16_t depth_mm = static_cast<uint16_t>(cam_z * 1000.0f);
```

**Benefits:**
- No camera extrinsics transformation needed
- No runtime intrinsics extraction
- Simpler coordinate system for planning
- Depth image directly in body frame

### Depth Image Processing Pipeline

1. **VK180 SDK Output**: Receive point cloud in body frame from stereo processing

2. **Distance Filtering**: Remove points too close to body center
   ```cpp
   if (distance < 0.4m) continue;  // Skip near-field points
   ```

3. **Virtual Camera Projection**: Transform body point to virtual camera frame
   ```cpp
   cam_x = body_y;    // Virtual camera orientation
   cam_y = -body_z;
   cam_z = body_x;
   ```

4. **Pinhole Projection**: Project to 2D image
   ```cpp
   u = fx * cam_x/cam_z + cx;
   v = fy * cam_y/cam_z + cy;
   depth_mm = cam_z * 1000;  // Convert to millimeters
   ```

5. **Z-Buffer**: Keep closest depth for each pixel (handle occlusions)

## Integration with FUEL Planner

### Launch Configuration

The FUEL planner is configured to use VK180 topics directly:

```bash
roslaunch vk180-ros vk180_exploration_flight.launch
```

**Key Topic Mappings**:
```xml
<!-- Fused odometry from PX4 EKF2 -->
<arg name="odometry_topic" value="/mavros/local_position/odom"/>

<!-- Raw VIO pose (same as vision input to PX4) -->
<arg name="sensor_pose_topic" value="/mavros/vision_pose/pose"/>

<!-- Body-frame depth from virtual camera -->
<arg name="depth_topic" value="/vk180/depth/image_rect_raw"/>

<!-- World-frame point cloud -->
<arg name="cloud_topic" value="/vk180/pointcloud"/>
```

### Virtual Camera Intrinsics

Fixed parameters for the ideal pinhole model:
```xml
<arg name="fx" value="320.0"/>
<arg name="fy" value="320.0"/>
<arg name="cx" value="320.0"/>
<arg name="cy" value="240.0"/>
<arg name="max_vel" value="1.0"/>
<arg name="max_acc" value="1.5"/>
```

### Frame Convention

- **Depth image frame**: `base_link` (body frame)
- **Sensor pose frame**: `odom` → `base_link` (body pose in world)
- **Point cloud frame**: `odom` (world frame)

This eliminates the need for camera-to-body TF transforms.

2. **Provide TF Transform**: Camera optical frame to base_link transform from extrinsics.

## Configuration Parameters

### Launch Parameters

```xml
<!-- Frame transformation -->
<arg name="use_body_frame_transform" default="true" />

<!-- Depth filtering -->
<arg name="max_depth" default="10.0" />
<arg name="min_depth" default="0.1" />

<!-- Frame IDs -->
<arg name="odom_frame_id" default="odom" />
<arg name="base_frame_id" default="base_link" />
```

### Key Benefits of Body Frame Transformation

1. **Accurate Obstacle Detection**: Obstacles positioned relative to robot center
2. **Simplified Planning**: No need for additional TF lookups during planning
3. **Consistent Coordinate System**: Odometry and depth in same reference frame
4. **Improved Mapping**: Better alignment between motion and sensor data

## Testing and Validation

### 1. Verify Extrinsics Extraction
```bash
# Run extrinsics test
./devel/lib/vk180_ros/depth_conversion_test

# Expected output: Transformation matrix and conversion examples
```

### 2. Check Depth Topics
```bash
# Start enhanced integration
roslaunch vk180_ros enhanced_planner_integration.launch

# Verify topics
rostopic list | grep -E "(vk180)"
rostopic echo /vk180/depth -n 1
```

### 3. Visualize in RViz
```bash
# Add depth displays in RViz:
# - /vk180/depth (for planner)
# - /grid_map/depth (for planner)
```

### 4. Integration Test with Planner
```bash
# Start VK180 integration
roslaunch vk180_ros enhanced_planner_integration.launch

# Start EGO planner (update topics as needed)
roslaunch ego_planner single_run_in_sim.launch
```

## Troubleshooting

### Common Issues

1. **No Extrinsics Extracted**
   - Ensure VK180 is streaming `S1/stereo1_l`
   - Check eCAL connection
   - Verify VK180 SDK installation

2. **Depth Values Too Large/Small**
   - Adjust `max_depth` and `min_depth` parameters
   - Check stereo baseline value in code
   - Verify disparity data quality

3. **Frame Transformation Issues**
   - Verify extrinsics extraction succeeded
   - Check transformation matrix values
   - Compare with VK180 calibration data

4. **Planner Integration Problems**
   - Ensure planner expects 16UC1 depth format
   - Verify frame_id consistency  
   - Check topic remapping in launch files

## Files Modified/Created

- `depth_conversion_test.cpp` - Extrinsics extraction test
- `enhanced_planner_integration.cpp` - Complete integration node
- `launch/enhanced_planner_integration.launch` - Launch configuration
- `CMakeLists.txt` - Build configuration updates

## Next Steps

1. **Test with Real Hardware**: Validate on actual VK180 setup
2. **Performance Optimization**: Profile depth conversion performance
3. **Parameter Tuning**: Optimize depth filtering and transformation
4. **Planner Configuration**: Fine-tune EGO planner for VK180 data
5. **Documentation**: Add RViz config and diagnostic tools