# How to Trigger VK180 Exploration

## Overview
The VK180 exploration system uses the FUEL (Fast UAV ExpLoration) framework. To trigger exploration, you need to use the **3D Nav Goal** tool in RViz.

## Prerequisites
1. Build the waypoint_generator package (already done):
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Starting the System

### 1. Launch the exploration system:
```bash
roslaunch vk180_ros vk180_exploration_flight.launch use_rviz:=true
```

### 2. Wait for initialization:
- Wait for VK180 odometry to be ready
- Wait for PX4 connection
- Check that all nodes are running properly

## Triggering Exploration with RViz

### Using the 3D Nav Goal Tool

The system includes a custom RViz plugin called **"3D Nav Goal"** that allows you to set goals with height control.

#### In RViz:
1. **Load the tool:**
   - The tool should appear in the RViz toolbar
   - Look for "3D Nav Goal" (shortcut key: `G`)

2. **Set a goal to trigger exploration:**
   - Click the "3D Nav Goal" button (or press `G`)
   - Click on a point in the 3D view
   - Drag to set the yaw direction
   - **Important:** Set the Z height to a positive value (> 0)

#### Goal Height Meanings:
- **Z > 0:** Triggers exploration at that point
- **Z between 0 and -1:** Removes the last waypoint (manual mode only)
- **Z < -1:** Publishes all collected waypoints (manual mode only)

### Example Workflow:

1. Start the system with RViz:
   ```bash
   roslaunch vk180_ros vk180_exploration_flight.launch use_rviz:=true
   ```

2. In RViz:
   - Press `G` or click "3D Nav Goal" tool
   - Click on any location in the map
   - Set height to 1.0m or higher
   - The drone should start exploring!

3. Monitor progress:
   - Watch the drone trajectory in RViz
   - Explored areas show in colorful voxels
   - Unexplored areas appear in grey

## Configuration

The waypoint_generator is configured in the launch file:
```xml
<node pkg="waypoint_generator" name="waypoint_generator" type="waypoint_generator">
  <remap from="~odom" to="/mavros/local_position/odom"/>
  <remap from="~goal" to="/move_base_simple/goal"/>
  <param name="waypoint_type" value="point"/>
</node>
```

### Waypoint Types:
- **"point"** (default): Single goal triggers immediately
- **"manual"**: Collect multiple waypoints before publishing

## Topics

The waypoint_generator node:
- **Subscribes to:**
  - `/move_base_simple/goal` - Goals from RViz 3D Nav Goal tool
  - `/mavros/local_position/odom` - Odometry for initialization
  
- **Publishes to:**
  - `/waypoint_generator/waypoints` - Waypoint path for exploration
  - `/waypoint_generator/waypoints_vis` - Visualization in RViz

## Troubleshooting

### "No odom yet, waiting..."
- Wait for VK180 to start publishing odometry
- Check that `/mavros/local_position/odom` is publishing

### Exploration doesn't start:
- Make sure Z height > 0 when setting goal
- Check that exploration_manager node is running
- Verify that the drone is in OFFBOARD mode (for real hardware)

### Can't see 3D Nav Goal tool:
- Check that rviz_plugins is built: `rospack find rviz_plugins`
- Restart RViz
- Check RViz toolbar for the "3D Nav Goal" button

## Alternative: Command-line Trigger

If you prefer command-line:
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  frame_id: 'world'
pose:
  position: {x: 2.0, y: 0.0, z: 1.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

## References

- [FUEL Paper](https://arxiv.org/abs/2010.11561)
- [FUEL GitHub](https://github.com/HKUST-Aerial-Robotics/FUEL)
- [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)
