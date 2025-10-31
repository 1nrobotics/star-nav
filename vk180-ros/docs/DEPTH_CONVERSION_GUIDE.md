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

### 2. Enhanced Planner Integration (`enhanced_planner_integration.cpp`)

**Purpose**: Complete VK180-to-ROS bridge with automatic depth frame transformation.

**Key Features**:
- Automatic extrinsics extraction from VK180 streams
- Real-time disparity-to-depth conversion  
- Camera-to-body frame transformation for all depth points
- RealSense-compatible depth format (16UC1 millimeters)
- Dual publishing: raw camera frame and transformed body frame

**Topics Published**:
- `/vk180/odom` - Odometry for planner (nav_msgs/Odometry)
- `/vk180/depth` - Body frame depth for planner (sensor_msgs/Image, 16UC1)
- `/vk180/disparity` - Disparity image
- `/vk180/pointcloud` - Combined point cloud from both cameras
- `/vk180/image_left` - Left camera RGB image
- `/vk180/image_right` - Right camera RGB image

## Coordinate Frame Transformation

### Camera Frame to Body Frame

The transformation converts 3D points from the stereo camera optical frame to the robot body frame:

```cpp
// Extract from VK180 sensor extrinsics
Eigen::Vector3d body_position;     // Translation camera -> body
Eigen::Quaterniond body_rotation;  // Rotation camera -> body

// Transform 3D point
Eigen::Vector3d camera_point(x, y, z);
Eigen::Vector3d body_point = body_rotation * camera_point + body_position;
```

### Depth Image Processing Pipeline

1. **Disparity to Depth**: Convert stereo disparity to metric depth
   ```cpp
   depth_m = (baseline * fx) / disparity
   ```

2. **Pixel to 3D Camera Point**: Use camera intrinsics
   ```cpp
   x_cam = (pixel_x - cx) * depth / fx
   y_cam = (pixel_y - cy) * depth / fy  
   z_cam = depth
   ```

3. **Camera to Body Transform**: Apply extrinsics transformation
   ```cpp
   body_point = R_cam_to_body * camera_point + t_cam_to_body
   ```

4. **Body Frame Distance**: For occupancy mapping
   ```cpp
   body_distance = body_point.norm()  // Distance from robot center
   ```

## Integration with EGO Planner

### Method 1: Direct Replacement (Recommended)

Replace the RealSense depth input with VK180 body frame depth:

1. **Start Enhanced Integration**:
   ```bash
   roslaunch vk180_ros enhanced_planner_integration.launch
   ```

2. **Update Planner Launch**: Replace RealSense topics in planner launch file:
   ```xml
   <!-- Before (RealSense) -->
   <remap from="/vk180/odom" to="/camera/odom/sample" />
   <remap from="/vk180/depth" to="/camera/depth/image_rect_raw" />
   
   <!-- After (VK180 with body frame) -->
   <!-- Topics already published correctly by enhanced_planner_integration -->
   ```

3. **Frame Configuration**: The depth is already in body frame, so planner can use directly.

### Method 2: TF-based Integration

If planner expects camera frame depth with TF transforms:

1. **Publish Camera Frame Depth**:
   ```cpp
   // Set use_body_frame_transform = false
   ```

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