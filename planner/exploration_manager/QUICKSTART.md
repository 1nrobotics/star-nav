# Exploration Manager - Implementation Summary

## Overview
Successfully created an autonomous exploration system using FUEL-inspired frontier-based exploration integrated with your existing VK180 + PX4 + EGO Planner setup.

## What Was Created

### 1. Package Structure
```
/home/yc/catkin_ws/src/planner/exploration_manager/
├── include/exploration_manager/
│   ├── frontier_finder.h           # Frontier detection algorithm
│   └── exploration_fsm.h           # Exploration state machine
├── src/
│   ├── frontier_finder.cpp         # Frontier implementation
│   ├── exploration_fsm.cpp         # FSM implementation
│   └── exploration_node.cpp        # Main ROS node
├── launch/
│   ├── exploration.launch          # Exploration module only
│   └── vk180_exploration.launch    # Complete system integration
├── CMakeLists.txt
├── package.xml
└── README.md                       # Comprehensive documentation
```

### 2. Key Components

#### Frontier Finder (`frontier_finder.h/cpp`)
- **Purpose**: Detects boundaries between known free space and unknown regions
- **Key Methods**:
  - `searchFrontiers()`: Finds all frontier cells in the map
  - `clusterFrontiers()`: Groups frontier cells into explorable regions
  - `getBestFrontier()`: Selects optimal frontier based on distance and size
  - `computeViewpoints()`: Generates observation poses around frontiers
- **Integration**: Uses GridMap from plan_env for occupancy data

#### Exploration FSM (`exploration_fsm.h/cpp`)
- **Purpose**: Manages exploration lifecycle and coordinates with EGO planner
- **States**:
  1. INIT - Initialize and wait for odometry
  2. WAIT_TRIGGER - Wait for user to start exploration
  3. SEARCH_FRONTIER - Find and select best frontier
  4. PLAN_TO_FRONTIER - Publish goal to trajectory planner
  5. EXEC_TRAJ - Monitor progress and replan if needed
  6. FINISH - Hover when exploration complete

#### Main Node (`exploration_node.cpp`)
- Initializes ExplorationFSM
- Runs with AsyncSpinner for concurrent callbacks

### 3. Launch Files

#### `vk180_exploration.launch` (Complete System)
Launches all required nodes:
- VK180 planner bridge (sensor fusion)
- PX4 controller (px4ctrl)
- EGO planner (trajectory generation)
- Exploration manager (frontier-based exploration)

**Usage**:
```bash
roslaunch exploration_manager vk180_exploration.launch
```

#### `exploration.launch` (Module Only)
Launches just the exploration node for testing.

### 4. ROS Interface

**Subscribed Topics**:
- `/mavros/local_position/odom` - Fused odometry from PX4 EKF2
- `/vk180/pointcloud` - Point cloud from VK180 sensor
- `/move_base_simple/goal` - Trigger exploration (from RViz 2D Nav Goal)
- `/exploration/finish` - Manual stop command

**Published Topics**:
- `/drone_0_planning/pos_cmd` - Position commands to px4ctrl
- `/exploration/state` - Current FSM state (for monitoring)
- `/exploration/frontiers` - Frontier markers (for RViz)
- `/exploration/frontier_cells` - Frontier point cloud (for RViz)

### 5. Parameters

Key parameters in launch file:
```xml
<!-- Exploration Behavior -->
<param name="fsm/max_exploration_time" value="600.0"/>          <!-- 10 minutes -->
<param name="fsm/frontier_search_interval" value="3.0"/>        <!-- Re-search every 3s -->

<!-- Frontier Detection -->
<param name="exploration/frontier_search_radius" value="10.0"/>  <!-- Search radius (m) -->
<param name="exploration/frontier_min_cells" value="10"/>        <!-- Min frontier size -->
<param name="exploration/viewpoint_distance" value="2.5"/>       <!-- Distance from frontier -->
<param name="exploration/viewpoint_num" value="8"/>              <!-- Viewpoints per frontier -->

<!-- Safety -->
<param name="manager/max_vel" value="1.0"/>                      <!-- Max velocity (m/s) -->
<param name="manager/clearance_threshold" value="0.2"/>          <!-- Obstacle clearance (m) -->
```

## How It Works

1. **Initialization**: 
   - Exploration node starts and waits for odometry
   - Creates shared GridMap instance for occupancy data
   - Subscribes to VK180 point cloud and PX4 odometry

2. **Trigger**: 
   - User clicks "2D Nav Goal" in RViz OR sends goal message
   - FSM transitions to SEARCH_FRONTIER state

3. **Frontier Detection**:
   - Scans GridMap for cells that are:
     * Free space
     * Adjacent to unknown space
   - Clusters nearby frontier cells into regions
   - Generates viewpoints around each frontier

4. **Goal Selection**:
   - Evaluates all frontiers by cost = distance - size_gain
   - Selects frontier with lowest cost (close + large)
   - Picks best viewpoint from candidates

5. **Execution**:
   - Publishes PositionCommand to `/drone_0_planning/pos_cmd`
   - EGO planner generates collision-free trajectory
   - px4ctrl converts to low-level commands
   - PX4 executes flight

6. **Monitoring**:
   - Checks if goal reached (distance < 0.5m)
   - Periodically re-searches frontiers (every 3s)
   - Transitions to FINISH when no frontiers remain

7. **Completion**:
   - Hovers in place when exploration complete
   - Can be re-triggered with new 2D Nav Goal

## Integration with Existing System

### Preserved Components
- ✅ VK180 sensor and planner bridge
- ✅ PX4 flight controller and px4ctrl
- ✅ MAVROS communication
- ✅ EGO planner trajectory generation
- ✅ Existing safety features

### New Components
- ➕ Frontier detection and clustering
- ➕ Exploration FSM for autonomous goal selection
- ➕ RViz visualization of frontiers
- ➕ High-level exploration logic

### Data Flow
```
VK180 Sensor
  ├─> /vk180/pointcloud ─────────> GridMap (plan_env)
  ├─> /vk180/odom_raw ────────────> VK180 Bridge
  └─> PX4 EKF2 Fusion

PX4 EKF2
  └─> /mavros/local_position/odom ─> ExplorationFSM & EGO Planner

ExplorationFSM
  ├─> Frontier Finder ────────────> Best Frontier
  ├─> /drone_0_planning/pos_cmd ──> EGO Planner
  └─> /exploration/* ─────────────> RViz Visualization

EGO Planner
  ├─> Trajectory Generation
  └─> /drone_0_planning/bspline ──> px4ctrl

px4ctrl
  └─> /mavros/setpoint_raw/attitude ─> PX4
```

## Next Steps

### 1. Build and Test
```bash
cd ~/catkin_ws
catkin_make

# Test in simulation first
roslaunch exploration_manager vk180_exploration.launch
```

### 2. Hardware Testing Checklist
- [ ] Verify sensor data rates (odometry 50Hz, point cloud 10-30Hz)
- [ ] Check odometry covariance (position < 0.1 m²)
- [ ] Test frontier detection with static obstacles
- [ ] Start with conservative parameters (max_vel=0.5 m/s)
- [ ] Have RC transmitter ready for manual override
- [ ] Test in open area (minimum 10m x 10m)

### 3. Parameter Tuning
Start conservative, then increase:
```xml
<!-- Phase 1: Safe Initial Test -->
<param name="manager/max_vel" value="0.5"/>
<param name="exploration/frontier_search_radius" value="5.0"/>

<!-- Phase 2: After Successful Tests -->
<param name="manager/max_vel" value="1.0"/>
<param name="exploration/frontier_search_radius" value="10.0"/>
```

### 4. Troubleshooting

**No frontiers found**:
- Check point cloud: `rostopic hz /vk180/pointcloud`
- Increase `frontier_search_radius`
- Decrease `frontier_min_cells`

**Drone doesn't move**:
- Check FSM state: `rostopic echo /exploration/state`
- Verify odometry: `rostopic echo /mavros/local_position/odom`
- Trigger exploration: Click 2D Nav Goal in RViz

**Erratic behavior**:
- Reduce `max_vel` and `max_acc`
- Increase `clearance_threshold`
- Check odometry quality (low covariance)

## Differences from FUEL

| Aspect | FUEL | This Implementation |
|--------|------|---------------------|
| **Odometry** | Perfect (simulation) | Real VK180 VIO + PX4 EKF2 |
| **Sensors** | Simulated depth | Real point cloud |
| **Planner** | FUEL's planner | EGO planner (proven) |
| **Control** | Direct commands | px4ctrl + PX4 |
| **FSM** | Complex multi-layer | Simplified for reliability |
| **Safety** | Minimal | Hardware-ready with timeouts |

## Key Adaptations for Real Hardware

1. **Sensor Fusion**: Uses PX4 EKF2 for robust odometry, not raw VIO
2. **Existing Infrastructure**: Integrates with proven px4ctrl and EGO planner
3. **Simplified FSM**: Single-layer state machine for reliability
4. **Safety Features**: Timeouts, distance checks, manual override support
5. **Modular Design**: Can disable exploration and use existing waypoint mode

## Files Modified/Created

**New Files** (12 total):
1. `/planner/exploration_manager/include/exploration_manager/frontier_finder.h`
2. `/planner/exploration_manager/include/exploration_manager/exploration_fsm.h`
3. `/planner/exploration_manager/src/frontier_finder.cpp`
4. `/planner/exploration_manager/src/exploration_fsm.cpp`
5. `/planner/exploration_manager/src/exploration_node.cpp`
6. `/planner/exploration_manager/launch/exploration.launch`
7. `/planner/exploration_manager/launch/vk180_exploration.launch`
8. `/planner/exploration_manager/CMakeLists.txt`
9. `/planner/exploration_manager/package.xml`
10. `/planner/exploration_manager/README.md`
11. `/planner/exploration_manager/QUICKSTART.md` (this file)

**Temporary** (removed):
- Cloned FUEL repository (used for reference, then removed to avoid conflicts)

## Build Status

✅ **Successfully Built**
```
[100%] Built target exploration_manager
[100%] Built target exploration_node
```

**Compiler Warnings**: Only minor warnings from traj_utils (existing code), no errors.

## References

- **FUEL Paper**: https://arxiv.org/abs/2010.11561
- **FUEL Repository**: https://github.com/HKUST-Aerial-Robotics/FUEL
- **EGO Planner**: https://github.com/ZJU-FAST-Lab/ego-planner

## Support

For issues or questions, refer to:
1. `/planner/exploration_manager/README.md` - Detailed documentation
2. ROS logs: `roscd exploration_manager && cat ~/.ros/log/latest/*.log`
3. Visualization: Launch RViz and add topics `/exploration/*`

---

**Status**: ✅ Complete - Ready for Testing
**Build**: ✅ Successful
**Integration**: ✅ Compatible with existing VK180 + PX4 + EGO system
