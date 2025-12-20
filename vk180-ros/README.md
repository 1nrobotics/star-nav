# VK180 ROS Integration Package# VK180 ROS Bridge



Comprehensive ROS package for integrating VK180 hardware with EGO Planner for autonomous navigation.This package provides a ROS bridge for VK180 eCAL odometry and disparity data, converting eCAL messages to standard ROS topics.



## 📁 Package Structure## Features



```- **Odometry Bridge**: Converts eCAL `Odometry3d` messages to ROS `nav_msgs/Odometry`

vk180-ros/- **Disparity Bridge**: Converts eCAL `Disparity` messages to ROS `sensor_msgs/Image` and `sensor_msgs/CameraInfo`

├── 📂 src/                          # Source code- **Transform Publishing**: Optionally publishes TF transforms for odometry

│   ├── 📂 nodes/                    # Main ROS nodes- **Configurable Topics**: Support for different eCAL topic names and frame IDs

│   │   ├── enhanced_planner_integration.cpp  # Main VK180+EGO integration

│   │   ├── ecal_ros_bridge_node.cpp          # Basic eCAL-ROS bridge## Published Topics

│   │   └── AllTopicReceiver.cpp              # Development debugging tool

│   └── 📂 utils/                    # Utility programs| Topic | Type | Description |

│       ├── odom.cpp                 # Odometry stream testing|-------|------|-------------|

│       ├── disparity.cpp            # Disparity stream testing  | `/vk180/odometry` | `nav_msgs/Odometry` | VIO odometry from eCAL |

│       ├── sensor_extrinsics.cpp    # Camera calibration extraction| `/vk180/disparity/image` | `sensor_msgs/Image` | Disparity/depth image from eCAL |

│       └── depth_conversion_test.cpp # Depth processing validation| `/vk180/disparity/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics and stereo info |

├── 📂 launch/                       # ROS launch files

│   ├── vk180_ego_planner_integration.launch  # Complete integration## TF Frames

│   ├── ecal_ros_bridge.launch               # Basic bridge only

│   ├── enhanced_planner_integration.launch  # Enhanced node only| Frame | Description |

│   └── planner_integration.launch           # Legacy integration|-------|-------------|

├── 📂 config/                       # Configuration files| `odom` | Odometry reference frame |

│   ├── vk180_planner_visualization.rviz     # RViz configuration| `base_link` | Robot base frame |

│   └── bridge_config.yaml                   # Bridge parameters| `camera_link` | Camera frame |

├── 📂 scripts/                      # Launch and utility scripts

│   ├── launch_vk180_planner.sh      # Full TMUX-based launch (recommended)## Parameters

│   ├── simple_launch.sh             # Simple testing launch

│   ├── analyze_disparity.py         # Disparity data analysis| Parameter | Type | Default | Description |

│   ├── visualize_disparity.py       # Disparity visualization|-----------|------|---------|-------------|

│   ├── check_tf.py                  # Transform debugging| `odom_frame_id` | string | `"odom"` | Odometry frame ID |

│   └── README.md                    # Script documentation| `base_frame_id` | string | `"base_link"` | Base frame ID |

├── 📂 docs/                         # Documentation| `camera_frame_id` | string | `"camera_link"` | Camera frame ID |

│   └── DEPTH_CONVERSION_GUIDE.md    # Depth processing guide| `ecal_odom_topic` | string | `"S1/vio_odom"` | eCAL odometry topic name |

├── CMakeLists.txt                   # Build configuration| `ecal_disparity_topic` | string | `"S1/stereo1_l/disparity"` | eCAL disparity topic name |

├── package.xml                      # ROS package manifest| `publish_tf` | bool | `true` | Whether to publish TF transforms |

└── README.md                        # This file

```## Usage



## 🚀 Quick Start### 1. Build the Package



### 1. Complete System Launch (Recommended)```bash

```bashcd ~/catkin_ws

# Launch with comprehensive TMUX interfacecatkin build vk180_ros

cd /home/yc/catkin_ws/src/vk180-rossource devel/setup.bash

./scripts/launch_vk180_planner.sh```

```

### 2. Launch the Bridge

### 2. Simple Testing Launch

```bash```bash

# Quick testing without TMUX complexity# Basic launch

cd /home/yc/catkin_ws/src/vk180-ros  roslaunch vk180_ros ecal_ros_bridge.launch

./scripts/simple_launch.sh

```# With custom parameters

roslaunch vk180_ros ecal_ros_bridge.launch odom_frame_id:=map base_frame_id:=robot_base

### 3. Component Testing```

```bash

# Test individual components### 3. Using Configuration File

rosrun vk180_ros sensor_extrinsics    # Check camera calibration

rosrun vk180_ros depth_conversion_test # Validate depth processing```bash

rosrun vk180_ros odom                 # Test VK180 odometry# Load parameters from config file

rosrun vk180_ros disparity            # Test VK180 disparityrosparam load $(rospack find vk180_ros)/config/bridge_config.yaml /ecal_ros_bridge/

```roslaunch vk180_ros ecal_ros_bridge.launch

```

## 🎯 Key Features

### 4. Monitor Topics

### Enhanced Planner Integration Node

- **VK180 Hardware Integration**: Direct eCAL communication with VK180```bash

- **Real-time Depth Processing**: Disparity to depth conversion with coordinate transforms# List active topics

- **EGO Planner Compatibility**: Seamless integration with trajectory planningrostopic list | grep vk180

- **Multi-format Publishing**: Raw depth, transformed depth, and disparity streams

- **Comprehensive Debugging**: Extensive logging and validation# Monitor odometry

rostopic echo /vk180/odometry

### Published Topics

```bash
/vk180/odom              # Odometry for EGO planner
/vk180/depth             # Depth images for obstacle mapping
/vk180/disparity         # Disparity image
/vk180/pointcloud        # Combined point cloud from both cameras
/vk180/image_left        # Left camera RGB image
/vk180/image_right       # Right camera RGB image
```

## FUEL Depth Compatibility

FUEL's `MapROS` depth pipeline expects a depth image (`sensor_msgs/Image`) in `16UC1` (millimeters) plus a pose stream. VK180 primarily provides a world-frame point cloud, so this repo adds a small projector node:

- Node: `vk180_cloud_to_depth`
- Subscribes: `/vk180/pointcloud` and a pose topic (recommended: `/mavros/local_position/pose`)
- Publishes: `/vk180/depth/image_rect_raw` (`16UC1`, mm)

This is wired into [vk180-ros/launch/vk180_exploration_flight.launch](vk180-ros/launch/vk180_exploration_flight.launch) so you can feed FUEL via `depth_topic:=/vk180/depth/image_rect_raw`.

## RViz (recommended displays)

For the full VK180 + FUEL exploration stack, the launch supports RViz:

```bash
roslaunch vk180_ros vk180_exploration_flight.launch use_rviz:=true
```

In RViz, keep:

- **Fixed Frame**: `world`
- **Depth Image**: `/vk180/depth/image_rect_raw` (projected from the VK180 cloud; `16UC1` mm)
- **Mapping clouds**: `/sdf_map/occupancy_all`, `/sdf_map/occupancy_local_inflate`
- **Planner markers**: `/planning_vis/trajectory`, `/planning_vis/frontier`, `/planning_vis/viewpoints`

If you want a VK180-only visualization preset, [vk180-ros/config/vk180_planner_visualization.rviz](vk180-ros/config/vk180_planner_visualization.rviz) now also includes `projected_depth_mm` for `/vk180/depth/image_rect_raw`.

## Dependencies

### Subscribed eCAL Streams

```bash### System Dependencies

S1/vio_odom                 # VK180 visual-inertial odometry- VK180 SDK (vilota)

S1/stereo1_l                # Left stereo camera for extrinsics- eCAL libraries

S1/stereo1_l/disparity      # Stereo disparity data- CapnProto

```

### ROS Dependencies

## 🔧 Configuration- roscpp

- nav_msgs

### Launch Parameters- sensor_msgs

```xml- geometry_msgs

<!-- Frame configuration -->- tf2_ros

<param name="odom_frame_id" value="odom" />

<param name="base_frame_id" value="base_link" />## Configuration Examples



<!-- Depth processing -->### Different eCAL Topics

<param name="use_body_frame_transform" value="true" />```yaml

<param name="max_depth" value="8.0" />ecal_odom_topic: "S0/vio_odom"        # Use sensor S0 instead of S1

<param name="min_depth" value="0.3" />ecal_disparity_topic: "S0/stereo1_l/disparity"

``````



### EGO Planner Integration### Custom Frame IDs

```xml```yaml

<!-- VK180 data topics -->odom_frame_id: "map"

```xml
<arg name="odometry_topic" value="/vk180/odom"/>
<arg name="depth_topic" value="/vk180/depth"/>
```

<!-- Camera intrinsics (auto-extracted from VK180) -->

<arg name="cx" value="320.0"/>### Disable TF Publishing

<arg name="cy" value="240.0"/>```yaml

<arg name="fx" value="400.0"/>publish_tf: false  # Useful when using robot_localization or other tf publishers

<arg name="fy" value="400.0"/>```

```

## Planner Integration

## 🛠️ Development

### EGO Planner Integration

### Building

```bashThe VK180 ROS bridge can directly integrate with EGO planner for real-time SLAM and path planning:

cd /home/yc/catkin_ws

catkin_make --pkg vk180_ros```bash

# or# Launch VK180 + EGO Planner integration

catkin build vk180_rosroslaunch vk180_ros planner_integration.launch

```

# Or launch bridge with planner support enabled

### Testingroslaunch vk180_ros ecal_ros_bridge.launch publish_to_planner:=true

```bash```

# Run individual components

**Integration Features:**

- ✅ **Direct odometry feed**: VK180 VIO → `vk180/odom`
- ✅ **Depth map conversion**: Disparity → depth in meters → `vk180/depth`  
- ✅ **Synchronized processing**: Message filters sync depth + odometry
- ✅ **Camera calibration**: Uses VK180 intrinsics for raycasting
- ✅ **Occupancy mapping**: Real-time 3D grid map for obstacle avoidance

**Published Topics for Planner:**

| Topic | Type | Description |
|-------|------|-------------|
| `/vk180/odom` | `nav_msgs/Odometry` | VK180 VIO odometry |
| `/vk180/depth` | `sensor_msgs/Image` | Depth image (16UC1, millimeters) |

1. **New ROS Nodes**: Add to `src/nodes/`

2. **Utility Programs**: Add to `src/utils/`**Camera Parameters:**

3. **Launch Files**: Add to `launch/`- fx, fy: 198.603 pixels

4. **Scripts**: Add to `scripts/`- cx, cy: 315.758, 189.992 pixels  

5. **Documentation**: Add to `docs/`- Baseline: 7.77 cm

- Resolution: 320×200 pixels

## 📊 System Architecture

### Integration Architecture

```

VK180 Hardware```

     ↓ (eCAL)VK180 Hardware

Enhanced Planner Integration Node    ↓ eCAL

     ↓ (ROS Topics)eCAL ROS Bridge

EGO Planner + RViz Visualization    ↓ ROS Topics

```EGO Planner (GridMap)

    ↓ Occupancy Grid

### Data FlowPath Planning & Control

1. **VK180** publishes stereo odometry and disparity via eCAL```

2. **Enhanced Integration Node** receives eCAL data and converts to ROS

3. **Coordinate Transformation** from camera frame to robot body frame### eCAL Connection Issues

4. **EGO Planner** consumes odometry and depth for path planning1. Ensure eCAL daemon is running: `ecal_daemon`

5. **RViz** provides real-time visualization and control interface2. Check available eCAL topics: `ecal_info`

3. Verify VK180 SDK installation: `/opt/vilota/`

## 📈 Performance

### ROS Topics Not Publishing

- **Odometry Rate**: ~30 Hz from VK1801. Check node status: `rosnode info /ecal_ros_bridge`

- **Disparity Rate**: ~30 Hz from VK180  2. Verify eCAL topics are active: `ecal_mon`

- **Depth Processing**: Real-time with ~10-12% valid pixels3. Check ROS master: `rostopic list`

- **Planning Update**: Real-time obstacle avoidance

- **Memory Usage**: Optimized Cap'n Proto handling### Build Issues

1. Ensure VK180 SDK paths are correct in CMakeLists.txt

## 🔍 Troubleshooting2. Check CapnProto installation

3. Verify ROS dependencies: `rosdep install --from-paths . --ignore-src -r -y`

### Common Issues

1. **VK180 Not Connected**: Check hardware and eCAL SDK## Original Test Executables

2. **No Depth Data**: Verify stereo cameras and extrinsics extraction

3. **Planning Failures**: Check odometry and depth topic ratesThe package also builds the original eCAL test executables:

4. **RViz Issues**: Verify frame transforms and topic remapping- `receive` - All topic receiver

- `odom` - Odometry-only receiver  

### Debug Commands- `disparity` - Disparity-only receiver

```bash- `sensor_extrinsics` - Extract and display sensor extrinsics from all streams

# Check VK180 connectivity

./devel/lib/vk180_ros/odom```bash

# Run original test programs

# Verify depth processingrosrun vk180_ros odom

./devel/lib/vk180_ros/depth_conversion_testrosrun vk180_ros disparity

rosrun vk180_ros receive

# Check transforms

rosrun tf tf_echo world base_link# Extract sensor extrinsics information

rosrun vk180_ros sensor_extrinsics

# Monitor system health```

rostopic echo /rosout | grep ERROR

```### Sensor Extrinsics Utility



## 📝 DependenciesThe `sensor_extrinsics` utility automatically scans for available streams and extracts:

- **Image Streams**: Body frame and IMU frame transforms from camera sensors

- **ROS Noetic** (Ubuntu 20.04)- **Disparity Streams**: Stereo camera rotation matrices for frame alignment  

- **VK180 eCAL SDK** (Vilota installation)- **Extrinsic Streams**: Dedicated sensor calibration data

- **OpenCV** (camera processing)

- **Eigen3** (coordinate transformations)This is useful for:

- **EGO Planner** (path planning)- Understanding camera mounting orientations

- Debugging coordinate frame transformations

## 🤝 Contributing- Validating stereo camera calibration

- Setting up proper TF transforms for ROS integration
1. Follow the organized directory structure
2. Add comprehensive documentation for new features
3. Include appropriate launch files and test scripts
4. Update this README for significant changes

## 📄 License

See main repository license.