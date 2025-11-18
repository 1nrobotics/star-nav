# Lite3 Quadruped Integration - System Overview

## Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         VK180 Sensor                                │
│  - Stereo Cameras (Point Cloud)                                    │
│  - VIO (Visual-Inertial Odometry)                                  │
└─────────────┬───────────────────────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    vk180_planner_bridge                             │
│  - Publishes: /vk180/pointcloud                                     │
│  - Publishes: /vk180/odom_raw                                       │
└─────────────┬───────────────────────────────────────────────────────┘
              │
              ├──────────────────────┬────────────────────────┐
              ▼                      ▼                        ▼
    ┌──────────────────┐   ┌──────────────────┐   ┌──────────────────┐
    │  Point Cloud     │   │   Odometry       │   │   Odometry       │
    │  /vk180/         │   │   /vk180/        │   │   /vk180/        │
    │  pointcloud      │   │   odom_raw       │   │   odom_raw       │
    └────────┬─────────┘   └────────┬─────────┘   └────────┬─────────┘
             │                      │                        │
             ▼                      │                        │
    ┌──────────────────┐           │                        │
    │   EGO Planner    │◄──────────┘                        │
    │  - Obstacle Map  │                                     │
    │  - Path Search   │                                     │
    │  - B-spline Opt  │                                     │
    └────────┬─────────┘                                     │
             │                                               │
             ▼                                               │
    ┌──────────────────┐                                     │
    │ Trajectory       │                                     │
    │ Server           │                                     │
    │ /drone_0_        │                                     │
    │ planning/bspline │                                     │
    └────────┬─────────┘                                     │
             │                                               │
             ▼                                               │
    ┌──────────────────────────────────────────────┐        │
    │        Position Command                      │        │
    │    /drone_0_planning/pos_cmd                 │        │
    │                                              │        │
    │  - position: {x, y, z}       [m]            │        │
    │  - velocity: {x, y, z}       [m/s]          │        │
    │  - acceleration: {x, y, z}   [m/s²]         │        │
    │  - yaw: float                [rad]          │        │
    │  - yaw_dot: float            [rad/s]        │        │
    └────────┬─────────────────────────────────────┘        │
             │                                               │
             ▼                                               │
    ┌──────────────────────────────────────────────┐        │
    │    Lite3 Velocity Controller                 │◄───────┘
    │                                              │
    │  Control Law:                                │
    │  v_cmd = v_desired + Kp*(p_des - p_curr)    │
    │                                              │
    │  Features:                                   │
    │  - Velocity feedforward                      │
    │  - Position feedback (Kp=0.5)                │
    │  - Exponential smoothing (α=0.3)             │
    │  - Velocity limiting (1.0 m/s)               │
    │  - Safety timeouts                           │
    └────────┬─────────────────────────────────────┘
             │
             ▼
    ┌──────────────────────────────────────────────┐
    │         Velocity Command                     │
    │         /lite3/cmd_vel                       │
    │                                              │
    │  - linear: {x, y, z=0}  [m/s]               │
    │  - angular: {x=0, y=0, z}  [rad/s]          │
    └────────┬─────────────────────────────────────┘
             │
             ▼
    ┌──────────────────────────────────────────────┐
    │       Deep Robotics Lite3 SDK                │
    │       (Your integration bridge)              │
    └────────┬─────────────────────────────────────┘
             │
             ▼
    ┌──────────────────────────────────────────────┐
    │          Lite3 Quadruped Robot               │
    │                                              │
    │  🦾 🦾 🦾 🦾                                 │
    └──────────────────────────────────────────────┘
```

## Key Differences from Quadrotor Setup

### 1. No PX4/MAVROS
- Quadrotor: Uses px4ctrl + MAVROS for attitude control
- Lite3: Direct velocity control (no flight controller needed)

### 2. Height Control
- Quadrotor: Full 3D position/velocity control
- Lite3: Robot handles height internally, only X/Y velocity commanded

### 3. Velocity Limits
- Quadrotor: 1.0-2.5 m/s typical
- Lite3: 0.8-1.0 m/s (conservative for quadruped locomotion)

### 4. Control Frequency
- Quadrotor: High-rate attitude control (100-400 Hz)
- Lite3: Velocity control at 50 Hz (sufficient for legged locomotion)

### 5. Waypoint Spacing
- Quadrotor: 5m waypoint spacing
- Lite3: 3m waypoint spacing (shorter range, slower speeds)

## Controller Tuning

### Position Gain (`position_gain`)
```
Higher (0.7-1.0): More aggressive tracking, faster convergence
Lower (0.3-0.4): Smoother motion, less overshoot
Default: 0.5 (balanced)
```

### Velocity Smoothing (`velocity_alpha`)
```
Higher (0.5-0.8): More responsive, less smoothing
Lower (0.1-0.3): More smoothing, less jittery
Default: 0.3 (smooth for quadruped)
```

### Max Velocities
```
max_linear_vel: 1.0 m/s (can push to 1.5 m/s if needed)
max_angular_vel: 1.0 rad/s (yaw rotation limit)
```

## Testing Workflow

1. **Start system:**
   ```bash
   roslaunch lite3_ctrl lite3_autonomous_nav.launch use_rviz:=true
   ```

2. **Monitor topics:**
   ```bash
   # Check planner commands
   rostopic echo /drone_0_planning/pos_cmd
   
   # Check velocity output
   rostopic echo /lite3/cmd_vel
   
   # Check odometry
   rostopic echo /vk180/odom_raw
   ```

3. **Set goal in RViz:**
   - Use "2D Nav Goal" tool
   - Click desired position
   - Robot plans path and follows

4. **Check controller status:**
   ```bash
   # View controller logs
   rosnode info /lite3_velocity_controller
   
   # Check control rate
   rostopic hz /lite3/cmd_vel
   ```

## Integration with Lite3 SDK

You need to create a bridge between ROS and Lite3 SDK:

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from lite3_sdk import Lite3Robot  # Your SDK import

class Lite3ROSBridge:
    def __init__(self):
        self.robot = Lite3Robot()  # Initialize SDK
        rospy.Subscriber('/lite3/cmd_vel', Twist, self.cmd_vel_callback)
        
    def cmd_vel_callback(self, msg):
        # Convert ROS Twist to Lite3 SDK command
        self.robot.set_velocity(
            vx=msg.linear.x,
            vy=msg.linear.y,
            vyaw=msg.angular.z
        )
```

## Safety Features

1. **Odometry Timeout (0.5s)**: Stops if no pose updates
2. **Command Timeout (1.0s)**: Stops if planner fails
3. **Velocity Limits**: Hard limits on commanded velocities
4. **Smooth Shutdown**: Zero velocity on node exit
5. **Exponential Smoothing**: Prevents jerky commands

## Next Steps

1. ✅ Controller implementation (done)
2. ⏳ Build and test in simulation
3. ⏳ Create Lite3 SDK bridge node
4. ⏳ Test with real hardware
5. ⏳ Tune parameters for optimal performance
6. ⏳ Add terrain-aware velocity adjustment
