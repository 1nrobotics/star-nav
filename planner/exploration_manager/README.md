# Exploration Manager

Simple waypoint-based autonomous exploration for the VK180 + PX4 + EGO Planner system.

## Overview

This package provides autonomous exploration capabilities for real drones using:
- **Waypoint-based Exploration**: Simple lawnmower pattern in a rectangular area
- **EGO Planner Integration**: Uses existing trajectory planner for safe navigation
- **Hardware Compatibility**: Works with VK180 vision sensor and PX4 flight controller

## Waypoint Algorithm

The waypoint-based exploration uses a simple but effective lawnmower pattern:

### Algorithm Overview
1. **Rectangle Definition**: Define exploration area as a rectangle centered at (center_x, center_y) with specified width and height
2. **Line Generation**: Create parallel lines spaced by `line_spacing` distance
3. **Alternating Direction**: Alternate direction on each line (left-to-right, right-to-left)
4. **Height Consistency**: All waypoints maintain constant altitude (center_z)
5. **Sequential Execution**: Visit waypoints in order, moving to next after timeout or completion

### Parameters
- `center_x`, `center_y`, `center_z`: Rectangle center and flight altitude
- `width`, `height`: Rectangle dimensions
- `line_spacing`: Distance between parallel lines
- `waypoint_timeout`: Maximum time to spend at each waypoint

### Example Pattern
For a 4m x 4m rectangle with 1m line spacing:
```
Waypoint 1: (center_x - 2, center_y - 2, center_z)  ←
Waypoint 2: (center_x - 1, center_y - 2, center_z)  ←
Waypoint 3: (center_x + 0, center_y - 2, center_z)  ←
Waypoint 4: (center_x + 1, center_y - 2, center_z)  ←
Waypoint 5: (center_x + 2, center_y - 2, center_z)  ←
Waypoint 6: (center_x + 2, center_y - 1, center_z)  →
Waypoint 7: (center_x + 1, center_y - 1, center_z)  →
Waypoint 8: (center_x + 0, center_y - 1, center_z)  →
Waypoint 9: (center_x - 1, center_y - 1, center_z)  →
Waypoint 10: (center_x - 2, center_y - 1, center_z) →
...continues for all lines
```

### Advantages
- **Predictable**: Deterministic coverage pattern
- **Simple**: Easy to understand and debug
- **Complete**: Systematic area coverage
- **Safe**: No complex frontier analysis
- **Configurable**: Easy parameter tuning

## Architecture

```
exploration_manager/
├── include/
│   └── exploration_manager/
│       ├── waypoint_exploration.h    # Main exploration class
│       └── waypoint_generator.h      # Waypoint generation utilities
├── src/
│   ├── waypoint_exploration.cpp      # Main exploration implementation
│   ├── waypoint_generator.cpp        # Waypoint generation implementation
│   └── exploration_node.cpp          # Main node
├── launch/
│   └── vk180_exploration.launch      # Complete system launch
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
- Exploration manager (waypoint-based exploration by default)

### Exploration Modes

The system supports two exploration modes:

#### Waypoint Mode (Default)
```xml
<param name="fsm/exploration_mode" value="waypoint"/>
```
- Generates systematic lawnmower pattern
- Covers rectangular area completely
- Simple and predictable behavior
- Good for structured environments

#### Frontier Mode
```xml
<param name="fsm/exploration_mode" value="frontier"/>
```
- Detects unknown boundaries dynamically
- Adapts to environment layout
- More complex but potentially more efficient
- Better for irregular spaces

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

### RViz Visualization

Launch with RViz to visualize the exploration process:

```bash
roslaunch exploration_manager vk180_exploration.launch use_rviz:=true
```

RViz displays:
- **Waypoints**: Blue markers showing planned waypoint path
- **Current Goal**: Green arrow showing active waypoint/goal
- **Drone Path**: Blue line showing executed trajectory
- **Occupancy Map**: Red/green map showing explored space
- **Frontiers**: Yellow markers (frontier mode only)

### Monitor Exploration

```bash
# View current FSM state
rostopic echo /exploration/state

# Waypoint mode topics
rostopic echo /exploration/waypoints

# Frontier mode topics (when using frontier mode)
rostopic echo /exploration/frontiers
rostopic echo /exploration/frontier_cells
```

## Complete Exploration Process

### 1. System Startup
1. **Launch**: `roslaunch exploration_manager vk180_exploration.launch`
2. **Sensors Initialize**: VK180 provides VIO odometry and point cloud
3. **PX4 Connects**: Flight controller establishes connection
4. **EGO Planner Ready**: Trajectory planner initializes with map parameters
5. **Exploration Manager**: FSM starts in INIT state, waits for odometry

### 2. Exploration Trigger
1. **User Input**: Place 2D Nav Goal in RViz or publish to `/move_base_simple/goal`
2. **FSM Transition**: Moves from WAIT_TRIGGER to exploration mode
3. **Mode Selection**: Based on `fsm/exploration_mode` parameter:
   - **Waypoint Mode**: Generate lawnmower pattern
   - **Frontier Mode**: Search for unknown boundaries

### 3. Waypoint Mode Execution
1. **Pattern Generation**: Create rectangular grid of waypoints
2. **Visualization**: Publish waypoints as MarkerArray to RViz
3. **Sequential Planning**: Plan trajectory to first waypoint using EGO planner
4. **Trajectory Execution**: Send position commands to PX4 controller
5. **Progress Monitoring**: Track waypoint completion with timeout
6. **Pattern Continuation**: Move to next waypoint when current completes
7. **Completion**: Finish when all waypoints visited or time limit reached

### 4. Safety and Monitoring
- **Geofence**: Stay within configured exploration radius
- **Obstacle Avoidance**: EGO planner handles dynamic obstacles
- **Timeout Protection**: Maximum time per waypoint prevents hanging
- **Manual Override**: RC transmitter always available for emergency
- **State Publishing**: Real-time FSM state for monitoring

### 5. Visualization in RViz
- **Waypoints**: Blue spheres showing planned exploration path
- **Current Goal**: Green arrow indicating active target
- **Executed Path**: Blue line showing actual drone trajectory
- **Occupancy Map**: Red areas (occupied), green areas (free)
- **Sensor Data**: Point cloud from VK180 vision sensor

### 6. System Shutdown
1. **Completion**: FSM reaches FINISH state
2. **Hover**: Drone maintains position
3. **Manual Stop**: Publish to `/exploration/finish` topic
4. **Emergency**: Use RC transmitter for immediate control

## Configuration

### Key Parameters

Edit in `launch/vk180_exploration.launch`:

```xml
<!-- Exploration mode selection -->
<param name="fsm/exploration_mode" value="frontier"/>          <!-- "frontier" or "waypoint" -->

<!-- General exploration behavior -->
<param name="fsm/max_exploration_time" value="600.0"/>          <!-- Max time (seconds) -->
<param name="fsm/waypoint_timeout" value="30.0"/>               <!-- Timeout per waypoint (seconds) -->

<!-- Frontier mode parameters -->
<param name="fsm/frontier_search_interval" value="60.0"/>       <!-- Re-search interval while executing -->
<param name="exploration/frontier_search_radius" value="10.0"/>  <!-- Search radius (m) -->
<param name="exploration/frontier_min_cells" value="10"/>        <!-- Min frontier size -->
<param name="exploration/viewpoint_distance" value="2.5"/>       <!-- Distance from frontier -->
<param name="exploration/viewpoint_num" value="8"/>              <!-- Viewpoints per frontier -->

<!-- Waypoint mode parameters -->
<param name="exploration/center_x" value="0.0"/>                <!-- Rectangle center X -->
<param name="exploration/center_y" value="0.0"/>                <!-- Rectangle center Y -->
<param name="exploration/center_z" value="1.5"/>                <!-- Flight altitude -->
<param name="exploration/width" value="10.0"/>                  <!-- Rectangle width (m) -->
<param name="exploration/height" value="10.0"/>                 <!-- Rectangle height (m) -->
<param name="exploration/line_spacing" value="0.5"/>            <!-- Distance between lines (m) -->

<!-- Safety -->
<param name="manager/max_vel" value="1.0"/>                      <!-- Max velocity (m/s) -->
<param name="manager/clearance_threshold" value="0.2"/>          <!-- Obstacle clearance -->
```

## FSM States

The exploration FSM has the following states:

1. **INIT**: Initialize and wait for odometry
2. **WAIT_TRIGGER**: Wait for user to trigger exploration
3. **SEARCH_FRONTIER**: Find and cluster frontier regions (frontier mode)
4. **GENERATE_WAYPOINTS**: Generate lawnmower pattern waypoints (waypoint mode)
5. **PLAN_TO_FRONTIER**: Plan trajectory to best frontier viewpoint (frontier mode)
6. **PLAN_TO_WAYPOINT**: Plan trajectory to next waypoint (waypoint mode)
7. **EXEC_TRAJ**: Execute trajectory and monitor progress (frontier mode)
8. **EXEC_WAYPOINT**: Execute trajectory and monitor progress with timeout (waypoint mode)
9. **FINISH**: Exploration complete, hover in place

## Topics

### Subscribed
- `/mavros/local_position/odom` (nav_msgs/Odometry): Fused odometry from PX4
- `/vk180/pointcloud` (sensor_msgs/PointCloud2): Point cloud from VK180
- `/move_base_simple/goal` (geometry_msgs/PoseStamped): Trigger exploration
- `/exploration/finish` (std_msgs/Empty): Manual finish command

### Published
- `/drone_0_planning/pos_cmd` (quadrotor_msgs/PositionCommand): Position commands to PX4
- `/exploration/state` (std_msgs/String): Current FSM state
- `/exploration/waypoints` (visualization_msgs/MarkerArray): Waypoint visualization

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
