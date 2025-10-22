# VK180 EGO Planner Launch Scripts

This directory contains launch scripts for integrated VK180 and EGO Planner operation with comprehensive visualization and debugging capabilities.

## 🚀 Launch Scripts

### 1. Full TMUX Launch (Recommended)
```bash
./scripts/launch_vk180_planner.sh
```

**Features:**
- ✅ **Multi-pane tmux session** with organized windows
- ✅ **Automatic ROS environment setup** 
- ✅ **Real-time monitoring** of all system components
- ✅ **Built-in debugging tools** and commands
- ✅ **RViz visualization** with pre-configured displays
- ✅ **Navigation control interface**

**TMUX Windows:**
- 🖥️ **roscore** - ROS core system
- 🚁 **vk180_integration** - Main VK180 + EGO planner
- 📡 **sensor_monitor** - VK180 sensor debugging
- 🗺️ **planner_monitor** - EGO planner debugging  
- 📊 **topic_monitor** - ROS topic monitoring
- 🎯 **navigation** - Navigation goal setting
- 📝 **logs** - System logs and diagnostics

### 2. Simple Launch
```bash
./scripts/simple_launch.sh
```

**Features:**
- ✅ **Single terminal** launch
- ✅ **Quick testing** and validation
- ✅ **RViz visualization** included
- ✅ **Minimal setup** for basic operation

## 🎮 TMUX Controls

### Session Management
```bash
# Attach to existing session
tmux attach -t vk180_planner

# Detach from session (keep running)
Ctrl+b + d

# Kill session completely
tmux kill-session -t vk180_planner
```

### Window Navigation
```bash
# Next window
Ctrl+b + n

# Previous window  
Ctrl+b + p

# Go to specific window (0-6)
Ctrl+b + [number]

# List all windows
Ctrl+b + w
```

### Pane Management
```bash
# Split horizontally
Ctrl+b + "

# Split vertically
Ctrl+b + %

# Switch panes
Ctrl+b + arrow keys
```

## 📊 System Monitoring

### Key Topics to Monitor
```bash
# VK180 depth data
rostopic hz /grid_map/depth
rostopic echo /vk180/raw_depth

# VK180 odometry
rostopic echo /grid_map/odom
rostopic hz /grid_map/odom

# Planning output
rostopic echo /drone_0_planning/data_display
rostopic echo /drone_0_planning/bspline

# System diagnostics
rostopic echo /rosout | grep ERROR
```

### Debug Commands
```bash
# VK180 sensor extrinsics
./devel/lib/vk180_ros/sensor_extrinsics

# Depth conversion test
./devel/lib/vk180_ros/depth_conversion_test

# Enhanced integration test
./devel/lib/vk180_ros/enhanced_planner_integration

# ROS graph visualization
rqt_graph

# TF tree
rqt_tf_tree
```

## 🎯 Navigation Control

### Using RViz 2D Nav Goal
1. **Open RViz** (automatically launched)
2. **Select "2D Nav Goal" tool** from toolbar
3. **Click and drag** in RViz to set goal position and orientation
4. **Watch trajectory planning** in real-time

### Command Line Navigation
```bash
# Publish navigation goal
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "
header:
  frame_id: world
pose:
  position: {x: 5.0, y: 3.0, z: 1.5}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

## 📈 RViz Displays

The pre-configured RViz setup includes:

- 🗺️ **Occupancy Grid** - Real-time obstacle map
- 🎯 **Planned Path** - EGO planner trajectory
- 📷 **Depth Image** - VK180 raw depth visualization  
- 📡 **VK180 Odometry** - Robot pose visualization
- ☁️ **Point Clouds** - 3D obstacle representation
- 🎨 **Planning Markers** - Trajectory optimization data

## ⚠️ Troubleshooting

### Common Issues

**1. TMUX session already exists**
```bash
tmux kill-session -t vk180_planner
```

**2. VK180 not connected**
- Check VK180 hardware connection
- Verify eCAL SDK installation
- Test with: `./devel/lib/vk180_ros/odom`

**3. No depth data**
- Check VK180 stereo cameras
- Verify topics: `rostopic list | grep depth`
- Test extrinsics: `./devel/lib/vk180_ros/sensor_extrinsics`

**4. Planner not working**
- Check odometry input: `rostopic echo /grid_map/odom`
- Verify depth format: `rostopic echo /grid_map/depth`
- Check frame transforms: `rosrun tf tf_echo world base_link`

**5. RViz crashes**
- Restart RViz window: Go to `vk180_integration` window and restart
- Check RViz config: `/home/yc/catkin_ws/src/vk180-ros/config/vk180_planner_visualization.rviz`

### Log Analysis
```bash
# View ROS logs
tail -f ~/.ros/log/latest/rosout.log

# Check for errors
rostopic echo /rosout | grep ERROR

# System diagnostics
roswtf
```

## 🔧 Configuration

### Planner Parameters
Edit `/home/yc/catkin_ws/src/vk180-ros/launch/vk180_ego_planner_integration.launch`:

```xml
<!-- Depth processing -->
<param name="max_depth" value="8.0" />
<param name="min_depth" value="0.3" />

<!-- Planning limits -->
<arg name="max_vel" value="1.0" />
<arg name="max_acc" value="2.0" />

<!-- Map size -->
<arg name="map_size_x" value="50"/>
<arg name="map_size_y" value="25"/>
<arg name="map_size_z" value="3.0"/>
```

### VK180 Integration
The enhanced planner integration automatically:
- ✅ Extracts camera intrinsics from VK180 data
- ✅ Converts depth from camera to body frame
- ✅ Publishes in RealSense-compatible format
- ✅ Provides synchronized odometry and depth

## 📝 Development Notes

- **Frame Transformations**: Depth automatically converted from camera optical frame to robot body frame
- **Coordinate Systems**: All data in consistent world/odom/base_link frame hierarchy
- **Real-time Performance**: Optimized for low-latency planning and obstacle avoidance
- **Extensibility**: Easy to add new sensors or modify planning parameters

## 🎓 Usage Examples

### Basic Operation
```bash
# Start everything
./scripts/launch_vk180_planner.sh

# Navigate in RViz using 2D Nav Goal
# Monitor in tmux windows
```

### Development/Testing
```bash
# Quick test without tmux
./scripts/simple_launch.sh

# Debug specific components
./devel/lib/vk180_ros/depth_conversion_test
```

### Performance Analysis
```bash
# Check data rates
rostopic hz /grid_map/depth
rostopic hz /grid_map/odom

# Monitor system load
htop

# ROS performance
rqt_plot
```