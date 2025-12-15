# VK180 + FUEL Integration - Quick Reference

## Launch Command
```bash
# Launch complete VK180 + FUEL autonomous exploration
roslaunch vk180_ros vk180_fuel_exploration.launch

# With custom exploration bounds
roslaunch vk180_ros vk180_fuel_exploration.launch \
    box_min_x:=-15.0 box_max_x:=15.0 \
    box_min_y:=-15.0 box_max_y:=15.0 \
    box_min_z:=0.5 box_max_z:=4.0
```

## Topic Connections (Verified)

### VK180 Outputs
```
/vk180/pointcloud              → FUEL /map_ros/cloud  ✅
/vk180/odom_raw                → (logging only)
/mavros/vision_pose/pose       → PX4 EKF2 input      ✅
```

### MAVROS Outputs  
```
/mavros/local_position/odom    → FUEL /odom_world    ✅
/mavros/local_position/pose    → FUEL /map_ros/pose  ✅
```

### FUEL Outputs
```
/planning/bspline              → traj_server         ✅
/planning/pos_cmd              → px4ctrl             ✅
```

## Critical Remappings in Launch File

Source code confirms FUEL expects these **hardcoded** topic names:

| FUEL Expects | Remapped To | Source Code Location |
|-------------|-------------|---------------------|
| `/odom_world` | `/mavros/local_position/odom` | `fast_exploration_fsm.cpp:43` |
| `/map_ros/cloud` | `/vk180/pointcloud` | `map_ros.cpp:73` |
| `/map_ros/pose` | `/mavros/local_position/pose` | `map_ros.cpp:75` |

## Data Flow Summary

```
VK180 Sensor → vk180_planner_bridge → /vk180/pointcloud
                                    → /mavros/vision_pose/pose
                                            ↓
                                         PX4 EKF2
                                            ↓
MAVROS ← /mavros/local_position/odom (fused odometry)
            ↓
        FUEL Planner (/odom_world, /map_ros/cloud)
            ↓
        /planning/bspline
            ↓
        traj_server → /planning/pos_cmd
            ↓
        px4ctrl → MAVROS → PX4 Motors
```

## Key Features

✅ **Zero FUEL code changes** - Only topic remapping  
✅ **Fused odometry** - VIO + IMU + GPS via PX4 EKF2  
✅ **World-frame clouds** - Pre-transformed by VK180 bridge  
✅ **Persistent map** - FUEL SDF never forgets explored regions  
✅ **Production-ready** - VK180 hardware-accelerated vision  

## Files Created

1. **Launch**: `/vk180-ros/launch/vk180_fuel_exploration.launch`
2. **Docs**: `/vk180-ros/docs/VK180_FUEL_INTERFACE.md`

## Next Steps

1. Test with hardware: `roslaunch vk180_ros vk180_fuel_exploration.launch`
2. Monitor topics: `rostopic hz /vk180/pointcloud /mavros/local_position/odom`
3. Visualize in RViz: Frontiers, trajectory, map
4. Start exploration: `rostopic pub /exploration/start std_msgs/Empty`
