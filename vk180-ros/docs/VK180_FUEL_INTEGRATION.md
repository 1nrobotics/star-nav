# VK180 Integration with FUEL Exploration Planner

## Overview
This document summarizes the integration of AMD VK180 stereo cameras with the FUEL autonomous exploration planner for the star-nav drone system.

## Background

### Challenge
VK180 provides raw disparity data through a custom VK SDK, not standard ROS depth images like RealSense cameras. The FUEL planner expects:
- `sensor_msgs/Image` depth images in 16UC1 format (millimeters)
- `geometry_msgs/PoseStamped` camera pose in world frame
- `sensor_msgs/CameraInfo` with camera intrinsics
- Proper timestamp synchronization between depth and pose

### Key Differences: RealSense vs VK180

| Aspect | RealSense | VK180 |
|--------|-----------|-------|
| **ROS Integration** | Official `realsense-ros` package | Custom VK SDK wrapper |
| **Depth Output** | Direct 16UC1 depth images | Raw disparity (requires conversion) |
| **Camera Pose** | Published via TF automatically | Computed from body pose + extrinsics |
| **Intrinsics** | Published as CameraInfo | Extracted from disparity metadata |
| **Frame Alignment** | Pre-aligned RGB+Depth | Manual transformation needed |

## Architecture

### Data Flow Pipeline

```
VK180 Hardware
    ↓
VK SDK (eCAL)
    ↓
vk180_planner_bridge
    ├─→ Disparity + Image → Point Cloud (camera frame)
    ├─→ Point Cloud → Depth Image (pinhole projection)
    ├─→ MAVROS Odom → Camera Pose (PoseStamped)
    └─→ Intrinsics → CameraInfo
    ↓
FUEL Exploration Planner
    ├─→ MapROS: Depth unprojection + Occupancy mapping
    ├─→ Frontier detection
    └─→ Path planning
```

### Key Components

#### 1. vk180_planner_bridge Node
**File:** `vk180-ros/src/nodes/vk180_planner_bridge.cpp`

**Responsibilities:**
- Subscribe to VK180 disparity and image streams via VK SDK
- Subscribe to MAVROS fused odometry (`/mavros/local_position/odom`)
- Extract camera extrinsics from VK180 image metadata
- Generate combined point cloud from dual stereo cameras
- Convert camera-frame point cloud to depth image
- Republish MAVROS body pose as camera pose (PoseStamped)
- Publish camera intrinsics (CameraInfo)

**Published Topics:**
- `/vk180/depth/image_rect_raw` - Depth image for planner (16UC1, mm)
- `/vk180/camera_pose` - Camera pose in odom frame (PoseStamped)
- `/vk180/depth/camera_info` - Camera intrinsics
- `/vk180/pointcloud` - Combined point cloud in world frame

**Subscribed Topics:**
- VK180 eCAL streams: `S1/stereo1_l`, `S1/stereo2_r`, `S1/stereo1_l/disparity`, etc.
- `/mavros/local_position/odom` - PX4 fused odometry

#### 2. FUEL Planner Configuration
**File:** `vk180-ros/launch/vk180_exploration_flight.launch`

**Key Parameters:**
```xml
<arg name="odometry_topic" value="/mavros/local_position/odom"/>
<arg name="sensor_pose_topic" value="/vk180/camera_pose"/>
<arg name="depth_topic" value="/vk180/depth/image_rect_raw"/>
<arg name="cloud_topic" value="/invalid_topic"/>  <!-- Disabled to avoid double-fusion -->

<!-- Ideal camera model -->
<arg name="fx" value="320.0"/>
<arg name="fy" value="320.0"/>
<arg name="cx" value="320.0"/>
<arg name="cy" value="240.0"/>
```

## Implementation Details

### 1. Depth Image Generation
**Method:** `convertCameraPointCloudToDepthImage()`

**Process:**
1. Receive point cloud in camera frame from VK SDK
2. Apply distance filter (>0.4m) and FOV filter
3. Project each point using pinhole camera model:
   ```cpp
   u = fx * x/z + cx
   v = fy * y/z + cy
   depth_mm = z * 1000
   ```
4. Use Z-buffer to keep closest depth per pixel
5. Publish as 640x480 16UC1 image

**Frame:** `camera_link` (matches TF tree: odom → base_link → camera_link)

### 2. Camera Pose Publishing
**Method:** `mavrosOdomCallback()`

**Process:**
1. Receive MAVROS body pose in `odom` frame
2. Republish as PoseStamped on `/vk180/camera_pose`
3. Same timestamp and frame as MAVROS
4. Camera offset from body center assumed negligible (for simplicity)

**Note:** Could apply `T_body_cam` transform for precise camera location, but current approach works well.

### 3. Coordinate Frames

**Frame Hierarchy:**
```
world (static)
  = odom (static identity TF)
    ← base_link (VK180 VIO + PX4 EKF2 fusion)
      ← camera_link (VK180 extrinsics)
```

**Planner Requirements:**
- **Camera pose frame_id:** `odom` (describes where camera is IN odom frame)
- **Depth image frame_id:** `camera_link` (depth values are IN camera's frame)
- **Planner transforms:** `world_point = camera_rotation * depth_point + camera_position`

### 4. Map Configuration Adjustments

#### Ground Height
**File:** `planner/exploration_manager/launch/algorithm.xml`

**Issue:** Default `ground_height=-1.0` but drone operates at z≈-1.35m

**Solution:** Changed to `ground_height=-2.0`
- Map Z range: -2.0 to 1.0 meters (3m height)
- Covers actual drone altitude

#### Occupancy Density
**Changes:**
1. `skip_pixel: 2 → 1` - Process every pixel instead of every 2nd (4x more points)
2. Fixed depth filtering bug in `map_ros.cpp` (row_ptr increment timing)
3. Enabled inflated occupancy visualization

**Result:** Denser occupancy map, better path planning success

## Bug Fixes

### 1. Depth Processing Bug in FUEL Planner
**File:** `planner/plan_env/src/map_ros.cpp`

**Original Code (Bug):**
```cpp
depth = (*row_ptr) * inv_factor;
row_ptr = row_ptr + skip_pixel_;
if (*row_ptr == 0 || depth > depth_filter_maxdist_)  // BUG: checking NEXT pixel!
    depth = depth_filter_maxdist_;
```

**Fixed Code:**
```cpp
uint16_t depth_raw = *row_ptr;
depth = depth_raw * inv_factor;
row_ptr = row_ptr + skip_pixel_;
if (depth_raw == 0 || depth > depth_filter_maxdist_)  // Correct: checking current pixel
    depth = depth_filter_maxdist_;
```

### 2. Message Type Mismatch
**Issue:** Planner expects `geometry_msgs/PoseStamped` but MAVROS publishes `nav_msgs/Odometry`

**Solution:** vk180_planner_bridge extracts pose from Odometry and republishes as PoseStamped

## Camera Intrinsics

### Ideal Camera Model
For simplicity and robustness, we use an ideal pinhole camera model:

```cpp
fx = fy = 320.0  // Focal length (pixels)
cx = 320.0       // Principal point X (image center)
cy = 240.0       // Principal point Y (image center)
width = 640      // Image width
height = 480     // Image height
```

**Field of View:** ~90 degrees (matches physical VK180 cameras)

**Scaling Factor:** 1000.0 (meters to millimeters)

## Testing and Validation

### Debug Logging Added
1. **vk180_planner_bridge:**
   - Depth image projection statistics
   - Camera pose publishing confirmation
   - Point cloud processing metrics

2. **FUEL MapROS:**
   - Depth/pose callback rate
   - Camera position validation
   - Depth point fusion statistics
   - Map bounds checking

### Verification Steps
1. Check depth image: `rostopic hz /vk180/depth/image_rect_raw`
2. Check camera pose: `rostopic echo /vk180/camera_pose`
3. Check occupancy map: `rostopic hz /sdf_map/occupancy_local`
4. Verify map contains camera: Look for "Camera position [x,y,z] is OUTSIDE map bounds" warning

## Performance Considerations

### Processing Load
- `skip_pixel=1`: ~4x more depth points than `skip_pixel=2`
- Trade-off: Denser map vs CPU usage
- VK180 hardware provides efficient disparity computation

### Memory Usage
- Map size: 10×10×3 meters at 0.1m resolution
- ~30,000 voxels for occupancy grid
- Acceptable for Jetson platforms

## Future Improvements

### Potential Enhancements
1. **Precise Camera Pose:** Apply `T_body_cam` transformation for exact camera location
2. **Adaptive skip_pixel:** Reduce processing when map is dense enough
3. **Multi-resolution mapping:** Coarse for distant areas, fine for nearby
4. **Temporal filtering:** Smooth depth over multiple frames
5. **Calibration refinement:** Use actual VK180 intrinsics instead of ideal model

### Known Limitations
1. Camera-body offset ignored (acceptable for small offsets)
2. Ideal camera model (no distortion correction)
3. Static obstacle assumption (no dynamic object tracking)

## Troubleshooting

### No Occupancy Data
**Symptoms:** Empty `/sdf_map/occupancy_local` topic

**Checks:**
1. Verify depth images published: `rostopic hz /vk180/depth/image_rect_raw`
2. Check camera pose: `rostopic echo /vk180/camera_pose`
3. Look for "Camera position OUTSIDE map bounds" warning
4. Verify depth callback: Look for "DepthPoseCallback" log messages
5. Check depth point count: "Fused X depth points into occupancy map"

**Common Causes:**
- Camera position outside map bounds (adjust `ground_height`)
- No valid depth data (check depth image statistics)
- Timestamp synchronization failure
- Message type mismatch

### Path Planning Fails
**Symptoms:** "No path to next viewpoint" errors

**Checks:**
1. Verify occupancy map populated
2. Check frontier detection: "Frontier: X" in logs
3. Verify inflated occupancy: `rostopic hz /sdf_map/occupancy_local_inflate`
4. Check if space is too cluttered (reduce `obstacles_inflation`)

### Sparse Occupancy Map
**Solutions:**
1. Reduce `skip_pixel` (currently 1, minimum value)
2. Increase `max_ray_length` for longer-range sensing
3. Verify depth image has valid data (not all zeros)
4. Check FOV filters aren't too restrictive

## References

### Related Files
- `vk180-ros/src/nodes/vk180_planner_bridge.cpp` - Main integration node
- `vk180-ros/launch/vk180_exploration_flight.launch` - Launch configuration
- `planner/exploration_manager/launch/algorithm.xml` - FUEL parameters
- `planner/plan_env/src/map_ros.cpp` - Occupancy mapping
- `planner/plan_env/src/sdf_map.cpp` - SDF map implementation

### Documentation
- [FUEL Planner GitHub](https://github.com/HKUST-Aerial-Robotics/FUEL)
- VK180 SDK Documentation
- MAVROS Documentation

---

**Last Updated:** December 20, 2025  
**Integration Status:** ✓ Functional - Depth fusion working, path planning operational
