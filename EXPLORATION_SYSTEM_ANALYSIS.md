# VK180 Autonomous Exploration System - Complete Flow & Potential Issues

## System Architecture Overview

```
VK180 Sensor → GridMap → FrontierFinder → ExplorationFSM → EGO Planner FSM → PX4 Controller → Drone
     ↓            ↓            ↓                  ↓                ↓                ↓
  /vk180/    Occupancy    Frontiers      /move_base_simple   B-spline      Attitude/Thrust
  pointcloud    Grid      Detection          /goal          Trajectory      Commands
```

## Complete Data Flow

### 1. Mapping Pipeline
```
VK180 Hardware
  ├─ Publishes: /vk180/pointcloud (sensor_msgs::PointCloud2)
  ├─ Publishes: /mavros/vision_pose/pose (VIO for PX4 EKF2)
  └─ Publishes: /mavros/local_position/odom (fused VIO+IMU)

GridMap (2 instances: EGO planner + Exploration)
  ├─ Subscribes: grid_map/cloud → /vk180/pointcloud
  ├─ Subscribes: grid_map/odom → /mavros/local_position/odom
  ├─ Updates: 3D occupancy grid (50x50x5m, 0.1m resolution)
  ├─ Publishes: grid_map/occupancy (visualization)
  └─ Publishes: grid_map/occupancy_inflate (for collision checking)
```

**⚠️ ISSUE #1: Duplicate GridMap Instances**
- **Problem**: Both EGO planner and exploration_manager create separate GridMap instances
- **Impact**: Double memory usage (~500MB each), double computation
- **Fix**: Exploration should query EGO planner's existing map instead
- **Code**: Line 18 in exploration_fsm.cpp creates redundant GridMap

### 2. Exploration State Machine
```
ExplorationFSM States:
  INIT → WAIT_TRIGGER → SEARCH_FRONTIER → PLAN_TO_FRONTIER → EXEC_TRAJ → FINISH
    ↓          ↓              ↓                    ↓                ↓          ↓
  Check     Wait for      Find          Publish goal to      Monitor    Complete
  odom    /exploration/  unexplored    /move_base_simple    reaching   or timeout
         trigger         regions            /goal            waypoint
```

**⚠️ ISSUE #2: Frame Inconsistency**
- **Problem**: Line 204 in exploration_fsm.cpp publishes goal in "world" frame
  ```cpp
  goal.header.frame_id = "world";
  ```
- **Expected**: Should be "odom" or verify EGO planner accepts "world"
- **Impact**: Goal transformation may fail, causing planner to ignore goals
- **Test**: Check static TF: world→odom exists in launch file ✓ (confirmed line 188)

### 3. Frontier Detection
```
FrontierFinder::searchFrontiers()
  ├─ Scans occupancy grid for unknown cells adjacent to free space
  ├─ Clusters frontier cells into regions
  ├─ Filters by geofence (25m radius default)
  ├─ Generates viewpoints around each frontier
  └─ Selects best frontier by distance + information gain
```

**⚠️ ISSUE #3: No Point Cloud Subscription Check**
- **Problem**: exploration_manager's GridMap subscribes to remapped topics but doesn't verify data flow
- **Launch file remapping** (line 164-165):
  ```xml
  <remap from="/exploration_node/odom" to="/mavros/local_position/odom"/>
  <remap from="/exploration_node/cloud" to="/vk180/pointcloud"/>
  ```
- **Actual subscription** (grid_map.cpp line 125):
  ```cpp
  indep_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("grid_map/cloud", 10, ...);
  ```
- **BUG**: Remapping doesn't match! Should be:
  ```xml
  <remap from="grid_map/cloud" to="/vk180/pointcloud"/>
  <remap from="grid_map/odom" to="/mavros/local_position/odom"/>
  ```

**⚠️ ISSUE #4: Frontier Search Computational Load**
- **Problem**: Searches 50x50x5m grid = 1.25 million voxels every 3 seconds
- **Impact**: May cause CPU spikes on Jetson Xavier NX
- **Mitigation**: Already limited by `frontier_search_interval` = 3.0s
- **Monitor**: Check CPU usage during flight: `htop` on drone

### 4. Goal Planning Flow
```
ExplorationFSM::planToFrontierState()
  ├─ Selects frontier viewpoint
  ├─ Publishes geometry_msgs::PoseStamped to /move_base_simple/goal
  └─ Transitions to EXEC_TRAJ state

EGO Planner FSM (flight_type=1, MANUAL_TARGET)
  ├─ waypointCallback() receives goal
  ├─ Calls planNextWaypoint(goal_pos)
  │   ├─ planGlobalTraj(): A* path on inflated map
  │   ├─ reboundReplan(): B-spline trajectory optimization
  │   └─ Collision checking with dynamic replanning
  ├─ Publishes: /drone_0_planning/bspline
  └─ State: WAIT_TARGET → GEN_NEW_TRAJ → EXEC_TRAJ → REPLAN_TRAJ (if needed)
```

**⚠️ ISSUE #5: No Confirmation of Goal Acceptance**
- **Problem**: exploration_fsm publishes goal but doesn't verify EGO planner received it
- **Failure mode**: If EGO planner is in EMERGENCY_STOP or not ready, goal ignored
- **Fix needed**: Subscribe to `/drone_0_planning/data_display` or add feedback topic
- **Workaround**: Monitor ROS logs for "[TRIG]" messages from EGO FSM

**⚠️ ISSUE #6: Race Condition on Rapid Goal Updates**
- **Problem**: Line 240 in exploration_fsm.cpp triggers new search every 3s during execution
  ```cpp
  if ((ros::Time::now() - last_frontier_search_time_).toSec() > frontier_search_interval_) {
    transitState(SEARCH_FRONTIER, "execTrajState");
  ```
- **Impact**: Could interrupt current trajectory with new goal before reaching old goal
- **Expected behavior**: EGO planner handles this via REPLAN_TRAJ state, but rapid switching wasteful
- **Recommendation**: Increase interval to 5-10s or only search when stuck

### 5. Trajectory Execution
```
Trajectory Server
  ├─ Subscribes: /drone_0_planning/bspline (traj_utils::Bspline)
  ├─ Evaluates B-spline at current time
  ├─ Generates position/velocity/acceleration commands
  └─ Publishes: /drone_0_planning/pos_cmd (quadrotor_msgs::PositionCommand)

PX4 Controller
  ├─ Subscribes: /drone_0_planning/pos_cmd
  ├─ Subscribes: /mavros/local_position/odom (for state feedback)
  ├─ Computes attitude + thrust from position error
  └─ Publishes: /mavros/setpoint_raw/attitude
```

**⚠️ ISSUE #7: Goal Reached Detection Too Coarse**
- **Problem**: Line 218 in exploration_fsm.cpp checks `dist_to_goal < 0.5`
  ```cpp
  if (dist_to_goal < 0.5) {
  ```
- **Impact**: For 0.8 m/s max velocity, drone may overshoot before detecting arrival
- **Better approach**: Check velocity near zero AND distance < threshold
- **Fix**:
  ```cpp
  if (dist_to_goal < 0.5 && odom_vel_.norm() < 0.2) {
  ```

**⚠️ ISSUE #8: Sleep in FSM Callback**
- **Problem**: Line 227 calls `ros::Duration(1.0).sleep()`
  ```cpp
  ros::Duration(1.0).sleep(); // Give time for map to update
  ```
- **Impact**: Blocks FSM callback timer, no state updates for 1 second
- **Risk**: Could cause issues if collision detected during sleep
- **Fix**: Use time-based state instead:
  ```cpp
  wait_start_time_ = ros::Time::now();
  transitState(WAIT_MAP_UPDATE, "execTrajState");
  // In WAIT_MAP_UPDATE state:
  if ((ros::Time::now() - wait_start_time_).toSec() > 1.0) {
    transitState(SEARCH_FRONTIER, "waitMapUpdateState");
  }
  ```

### 6. Control Loop
```
PX4 Autopilot (onboard flight controller)
  ├─ Receives: /mavros/setpoint_raw/attitude (100 Hz from px4ctrl)
  ├─ EKF2: Fuses /mavros/vision_pose/pose + IMU → local position estimate
  ├─ Attitude controller: Maintains commanded roll/pitch/yaw/thrust
  └─ Motor mixing: ESC signals to motors
```

## Critical Real Flight Issues

### **CRITICAL ISSUE #9: Missing Safety Geofence Validation**
- **Problem**: Geofence only applied at frontier selection, not at goal publishing
- **Risk**: If frontier moves due to map updates, could violate geofence
- **Fix needed** in exploration_fsm.cpp planToFrontierState():
  ```cpp
  // Validate geofence before publishing
  double dist_from_center = (target_pt_ - geofence_center_).norm();
  if (dist_from_center > max_exploration_radius_) {
    ROS_ERROR("[ExplorationFSM] Goal violates geofence! Searching again...");
    transitState(SEARCH_FRONTIER, "planToFrontierState");
    return;
  }
  ```

### **CRITICAL ISSUE #10: No Emergency Stop Handling**
- **Problem**: If user triggers emergency stop on transmitter (CH5/CH6), exploration continues
- **Impact**: exploration_fsm keeps publishing goals even when px4ctrl disabled
- **Fix**: Subscribe to `/mavros/state` and check `armed` status
  ```cpp
  if (!mavros_state_.armed || mavros_state_.mode != "OFFBOARD") {
    ROS_ERROR("[ExplorationFSM] Flight mode changed! Stopping exploration.");
    transitState(FINISH, "emergencyCheck");
  }
  ```

### **CRITICAL ISSUE #11: Map Initialization Race Condition**
- **Problem**: exploration_fsm starts immediately after launch, but GridMap needs time to build
- **Impact**: First frontier search may return empty map, ending exploration prematurely
- **Fix**: Check if map has received sufficient point clouds before starting
  ```cpp
  void ExplorationFSM::waitTriggerState() {
    if (!grid_map_->hasReceivedCloud()) {
      ROS_WARN_THROTTLE(2.0, "Waiting for point cloud data...");
      return;
    }
    if (exploration_triggered_) {
      transitState(SEARCH_FRONTIER, "waitTriggerState");
    }
  }
  ```

### **CRITICAL ISSUE #12: Topic Remapping Bug**
- **Location**: vk180_exploration.launch lines 164-165
- **Current (WRONG)**:
  ```xml
  <remap from="/exploration_node/odom" to="/mavros/local_position/odom"/>
  <remap from="/exploration_node/cloud" to="/vk180/pointcloud"/>
  ```
- **Should be**:
  ```xml
  <remap from="grid_map/odom" to="/mavros/local_position/odom"/>
  <remap from="grid_map/cloud" to="/vk180/pointcloud"/>
  ```
- **Impact**: Exploration manager's GridMap receives NO point cloud data!
- **Result**: Empty map → no frontiers → exploration ends immediately

## Parameter Validation Checklist

### Launch File Parameters to Verify:
- [x] `map_size_x/y/z`: 50x50x5m - reasonable for indoor/outdoor
- [x] `max_vel`: 0.8 m/s - safe for exploration
- [x] `max_acc`: 1.0 m/s² - conservative
- [ ] ⚠️ `local_update_range`: 30m must be >= max_exploration_radius (25m) ✓
- [ ] ⚠️ `obstacles_inflation`: 0.3m - CHECK drone size! (typical FPV: 0.25-0.3m diagonal)
- [x] `max_exploration_time`: 600s (10 min) - adjustable for mission

### Exploration Parameters:
- [x] `frontier_search_interval`: 3.0s - may increase to 5s
- [x] `frontier_search_radius`: 10.0m - good balance
- [x] `max_exploration_radius`: 25.0m - safe for testing
- [ ] ⚠️ **geofence_center must be set to takeoff position!** Currently (0, 0, 1.5)

## Testing Protocol

### Pre-Flight Checks:
1. ✅ Launch system: `roslaunch vk180_ros vk180_exploration.launch use_rviz:=true`
2. ⚠️ **Fix topic remapping first!** (CRITICAL ISSUE #12)
3. ✅ Verify topics:
   ```bash
   rostopic hz /vk180/pointcloud        # Should be ~10-30 Hz
   rostopic hz /mavros/local_position/odom  # Should be 30-50 Hz
   rostopic echo /exploration/state     # Should show "WAIT_TRIGGER"
   ```
4. ⚠️ Check GridMap receiving data:
   ```bash
   rostopic echo /grid_map/occupancy -n 1  # Should publish after ~5s
   ```
5. ✅ Enable PX4 controller: Switch CH5 and CH6 on transmitter
6. ✅ Trigger exploration:
   ```bash
   rostopic pub /exploration/trigger geometry_msgs/PoseStamped "{}"
   ```

### In-Flight Monitoring:
```bash
# Terminal 1: State monitoring
watch -n 0.5 rostopic echo /exploration/state

# Terminal 2: Goal tracking
rostopic echo /move_base_simple/goal

# Terminal 3: Distance to goal
rostopic echo /mavros/local_position/odom | grep -A3 "position"

# Terminal 4: Frontier visualization
rostopic echo /exploration/frontiers -n 1
```

### Debug Commands:
```bash
# If exploration not starting:
rostopic info /exploration/trigger
rostopic info /move_base_simple/goal

# If no frontiers found:
rostopic echo /grid_map/occupancy -n 1   # Check map exists
rosrun rviz rviz  # Visualize occupancy grid

# If EGO planner not receiving goals:
rostopic echo /move_base_simple/goal
rqt_graph  # Check topic connections

# Emergency stop:
rostopic pub /exploration/finish std_msgs/Empty "{}"
```

## Recommended Fixes Priority

### **MUST FIX BEFORE FLIGHT:**
1. ✅ **ISSUE #12**: Fix topic remapping in launch file (drone won't explore without map!)
2. ✅ **ISSUE #9**: Add geofence validation before publishing goals
3. ✅ **ISSUE #10**: Subscribe to MAVROS state for emergency detection
4. ✅ **ISSUE #11**: Add map initialization check

### **SHOULD FIX:**
5. ⚠️ **ISSUE #7**: Improve goal reached detection with velocity check
6. ⚠️ **ISSUE #8**: Replace sleep() with time-based state
7. ⚠️ **ISSUE #5**: Add goal acceptance confirmation

### **NICE TO HAVE:**
8. ⚠️ **ISSUE #1**: Share GridMap between EGO and exploration
9. ⚠️ **ISSUE #6**: Increase frontier search interval during execution
10. ⚠️ **ISSUE #4**: Profile CPU usage, optimize if needed

## Success Criteria
- [x] Exploration launches without errors
- [ ] GridMap receives point cloud (check after fix #12)
- [ ] Frontiers detected and visualized in RViz
- [ ] Goals published to /move_base_simple/goal
- [ ] EGO planner generates trajectories (check /drone_0_planning/bspline)
- [ ] Drone follows trajectory to frontier
- [ ] New frontiers searched after reaching goal
- [ ] Exploration completes or times out gracefully

## Post-Flight Analysis
1. Rosbag key topics:
   ```bash
   rosbag record /vk180/pointcloud /mavros/local_position/odom \
                 /exploration/frontiers /move_base_simple/goal \
                 /drone_0_planning/bspline /drone_0_planning/pos_cmd \
                 /exploration/state -O exploration_test.bag
   ```
2. Check exploration coverage using map_server
3. Analyze trajectory smoothness and replanning frequency
4. Verify geofence compliance throughout flight
