# Exploration Manager

Autonomous exploration module using FUEL-inspired frontier-based exploration for the VK180 + PX4 + EGO Planner system.

## Overview

This package provides autonomous exploration capabilities for real drones using:
- **Frontier Detection**: Identifies boundaries between known free space and unknown areas
- **Exploration FSM**: State machine that manages exploration lifecycle
- **EGO Planner Integration**: Uses existing trajectory planner for safe navigation
- **Hardware Compatibility**: Works with VK180 vision sensor and PX4 flight controller

## Architecture

```
exploration_manager/
├── include/
│   └── exploration_manager/
│       ├── frontier_finder.h       # Frontier detection
│       └── exploration_fsm.h       # Exploration state machine
├── src/
│   ├── frontier_finder.cpp         # Frontier implementation
│   ├── exploration_fsm.cpp         # FSM implementation
│   └── exploration_node.cpp        # Main node
├── launch/
│   ├── exploration.launch          # Exploration only
│   └── vk180_exploration.launch    # Complete system launch
├── CMakeLists.txt
└── package.xml
```

## Dependencies

### ROS Packages
- `roscpp`
- `pcl_ros`
- `plan_env` (from EGO planner)
- `plan_manage` (from EGO planner)
- `traj_utils` (from quadrotor_msgs)

### External Libraries
- Eigen3
- PCL (Point Cloud Library)

## Installation

1. **Clone and build:**
   ```bash
   cd ~/catkin_ws/src/planner
   # exploration_manager should already be here
   cd ~/catkin_ws
   catkin build exploration_manager
   ```

2. **Verify dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Usage

### Full System Launch

Launch complete autonomous exploration system:

```bash
roslaunch exploration_manager vk180_exploration.launch
```

This starts:
- VK180 planner bridge (sensor fusion)
- PX4 controller
- EGO planner (trajectory generation)
- Exploration manager (frontier-based exploration)

### Trigger Exploration

Use RViz 2D Nav Goal to trigger exploration:

```bash
# In RViz, click "2D Nav Goal" and place anywhere on the map
# Exploration will start automatically
```

Or use command line:

```bash
# Trigger exploration
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: 'odom'}, pose: {position: {x: 0, y: 0, z: 1}}}"

# Stop exploration manually
rostopic pub /exploration/finish std_msgs/Empty "{}"
```

### Monitor Exploration

```bash
# View current FSM state
rostopic echo /exploration/state

# View discovered frontiers
rostopic echo /exploration/frontiers

# View frontier cells
rostopic echo /exploration/frontier_cells
```

## Configuration

### Key Parameters

Edit in `launch/vk180_exploration.launch`:

```xml
<!-- Exploration behavior -->
<param name="fsm/max_exploration_time" value="600.0"/>          <!-- Max time (seconds) -->
<param name="fsm/frontier_search_interval" value="60.0"/>       <!-- Re-search interval while executing -->

<!-- Frontier detection -->
<param name="exploration/frontier_search_radius" value="10.0"/>  <!-- Search radius (m) -->
<param name="exploration/frontier_min_cells" value="10"/>        <!-- Min frontier size -->
<param name="exploration/viewpoint_distance" value="2.5"/>       <!-- Distance from frontier -->
<param name="exploration/viewpoint_num" value="8"/>              <!-- Viewpoints per frontier -->

<!-- Safety -->
<param name="manager/max_vel" value="1.0"/>                      <!-- Max velocity (m/s) -->
<param name="manager/clearance_threshold" value="0.2"/>          <!-- Obstacle clearance -->
```

## FSM States

The exploration FSM has the following states:

1. **INIT**: Initialize and wait for odometry
2. **WAIT_TRIGGER**: Wait for user to trigger exploration
3. **SEARCH_FRONTIER**: Find and cluster frontier regions
4. **PLAN_TO_FRONTIER**: Plan trajectory to best frontier viewpoint
5. **EXEC_TRAJ**: Execute trajectory and monitor progress
6. **FINISH**: Exploration complete, hover in place

## Topics

### Subscribed
- `/mavros/local_position/odom` (nav_msgs/Odometry): Fused odometry from PX4
- `/vk180/pointcloud` (sensor_msgs/PointCloud2): Point cloud from VK180
- `/move_base_simple/goal` (geometry_msgs/PoseStamped): Trigger exploration
- `/exploration/finish` (std_msgs/Empty): Manual finish command

### Published
- `/drone_0_planning/pos_cmd` (quadrotor_msgs/PositionCommand): Position commands to PX4
- `/exploration/state` (std_msgs/String): Current FSM state
- `/exploration/frontiers` (visualization_msgs/MarkerArray): Frontier visualization
- `/exploration/frontier_cells` (sensor_msgs/PointCloud2): Frontier cells

## Hardware Setup Checklist

Before running on real drone:

### 1. Verify Sensor Data
```bash
# Check VK180 odometry (30-60 Hz expected)
rostopic hz /vk180/odom_raw

# Check point cloud (10-30 Hz expected)
rostopic hz /vk180/pointcloud

# Check PX4 fused odometry (50 Hz expected)
rostopic hz /mavros/local_position/odom
```

### 2. Verify Odometry Quality
```bash
# Monitor odometry drift
rostopic echo /mavros/local_position/odom

# Check covariance values (should be low)
# position covariance < 0.1 m²
# velocity covariance < 0.05 m²/s²
```

### 3. Test in Simulation First
```bash
# Use Gazebo or AirSim before real flight
# Verify frontier detection works
# Check trajectory generation
# Monitor safety behaviors
```

### 4. Safety Precautions
- Always have RC transmitter ready for manual override
- Test in open area first (minimum 10m x 10m)
- Start with conservative parameters (max_vel=0.5 m/s)
- Use safety observer to monitor drone
- Set reasonable max_exploration_time (start with 5 minutes)

### 5. Tuning Parameters

Start conservative, then increase:
```xml
<!-- Initial safe values -->
<param name="manager/max_vel" value="0.5"/>
<param name="manager/max_acc" value="0.5"/>
<param name="exploration/frontier_search_radius" value="5.0"/>

<!-- After successful tests, increase gradually -->
<param name="manager/max_vel" value="1.0"/>
<param name="manager/max_acc" value="1.0"/>
<param name="exploration/frontier_search_radius" value="10.0"/>
```

## Differences from FUEL

This implementation adapts FUEL's exploration algorithm for real hardware:

| Feature | FUEL (Original) | This Implementation |
|---------|-----------------|---------------------|
| Odometry | Perfect (sim) | PX4 EKF2 fused VIO |
| Sensors | Simulated depth | Real VK180 point cloud |
| Planner | FUEL's planner | EGO planner (proven) |
| FSM | Complex multi-layer | Simplified single-layer |
| Safety | Minimal | Hardware-ready |

Key adaptations:
- Uses real sensor fusion (VK180 VIO + PX4 EKF2)
- Integrates with proven px4ctrl for low-level control
- Keeps existing EGO planner for trajectory generation
- Adds safety monitoring and timeout protection
- Simplified FSM for reliability

## Troubleshooting

### No frontiers found
- Check if point cloud is received: `rostopic hz /vk180/pointcloud`
- Verify map is updating: `rostopic echo /drone_0_planning/data_display`
- Increase `frontier_search_radius`
- Decrease `frontier_min_cells`

### Drone doesn't move
- Check FSM state: `rostopic echo /exploration/state`
- Verify odometry: `rostopic echo /mavros/local_position/odom`
- Check if exploration triggered: Send 2D Nav Goal in RViz
- Monitor PX4 controller: `rostopic echo /mavros/state`

### Erratic behavior
- Reduce `max_vel` and `max_acc`
- Increase `clearance_threshold`
- Check odometry covariance (may indicate poor localization)
- Verify point cloud quality (not too sparse or noisy)

### Compilation errors
```bash
# Check dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean build
cd ~/catkin_ws
catkin clean exploration_manager
catkin build exploration_manager
```

## References

- FUEL Paper: https://arxiv.org/abs/2010.11561
- FUEL Repository: https://github.com/HKUST-Aerial-Robotics/FUEL
- EGO Planner: https://github.com/ZJU-FAST-Lab/ego-planner

## License

Same as parent workspace

## Authors

Adapted from FUEL for VK180 + PX4 + EGO Planner system
