# Lite3 Simple Walk Test

Simple test scripts to verify basic locomotion of the Deep Robotics Lite3 quadruped robot.

## Overview

This test suite provides pre-programmed walking patterns to test the Lite3 robot's basic movement capabilities:
- Forward/backward walking
- Left/right strafing
- Turning (left/right)
- Circle walking
- Combined movements

## Prerequisites

1. **Lite3 Robot**: Powered on and ready
2. **Message Transformer**: The `message_transformer` package from Lite3_ROS must be built
3. **Network Connection**: Perception host must be connected to motion host via UDP
4. **Robot Mode**: Switch robot to **AUTO mode** using the app before testing

## Quick Start

### Option 1: Simple Forward Test (Recommended for first test)

```bash
# Terminal 1: Launch message transformer
roslaunch message_transformer message_transformer.launch

# Terminal 2: Run simple forward walk test
roslaunch lite3_ctrl lite3_walk_test_standalone.launch test_mode:=simple
```

The robot will:
1. Wait 3 seconds
2. Walk forward at 0.2 m/s for 5 seconds
3. Stop for 2 seconds

### Option 2: Full Test Sequence

```bash
# Terminal 1: Launch message transformer
roslaunch message_transformer message_transformer.launch

# Terminal 2: Run full walking test
roslaunch lite3_ctrl lite3_walk_test_standalone.launch test_mode:=full
```

The robot will perform:
1. Walk forward (3s)
2. Stop (1s)
3. Walk backward (3s)
4. Stop (1s)
5. Turn left (2s)
6. Stop (1s)
7. Turn right (2s)
8. Stop (1s)
9. Strafe left (2s)
10. Stop (1s)
11. Strafe right (2s)
12. Stop (1s)
13. Walk in circle (5s)
14. Stop (1s)

### Option 3: All-in-One Launch (if message_transformer path is correct)

```bash
# Launches both message_transformer and walk test
roslaunch lite3_ctrl lite3_walk_test.launch test_mode:=simple
```

## Manual Testing with rostopic

You can also manually test by publishing velocity commands:

```bash
# Terminal 1: Start message transformer
roslaunch message_transformer message_transformer.launch

# Terminal 2: Publish velocity command (forward 0.2 m/s at 10 Hz)
rostopic pub /cmd_vel geometry_msgs/Twist -r 10 "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### Velocity Command Reference

**Forward/Backward:**
```bash
# Forward at 0.3 m/s
rostopic pub /cmd_vel geometry_msgs/Twist -r 10 "linear: {x: 0.3, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"

# Backward at 0.3 m/s
rostopic pub /cmd_vel geometry_msgs/Twist -r 10 "linear: {x: -0.3, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"
```

**Strafe Left/Right:**
```bash
# Strafe left at 0.2 m/s
rostopic pub /cmd_vel geometry_msgs/Twist -r 10 "linear: {x: 0.0, y: 0.2, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"

# Strafe right at 0.2 m/s
rostopic pub /cmd_vel geometry_msgs/Twist -r 10 "linear: {x: 0.0, y: -0.2, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"
```

**Turn Left/Right:**
```bash
# Turn left at 0.5 rad/s
rostopic pub /cmd_vel geometry_msgs/Twist -r 10 "linear: {x: 0.0, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.5}"

# Turn right at 0.5 rad/s
rostopic pub /cmd_vel geometry_msgs/Twist -r 10 "linear: {x: 0.0, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: -0.5}"
```

**Stop:**
```bash
# Stop robot (IMPORTANT for emergency stop!)
rostopic pub /cmd_vel geometry_msgs/Twist -r 10 "linear: {x: 0.0, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"
```

## Monitoring Robot Status

```bash
# Check odometry
rostopic echo /leg_odom

# Check IMU data
rostopic echo /imu/data

# Check joint states
rostopic echo /joint_states

# Monitor command velocity being published
rostopic echo /cmd_vel

# Check message frequency
rostopic hz /leg_odom
rostopic hz /cmd_vel
```

## Safety Guidelines

⚠️ **IMPORTANT SAFETY NOTES:**

1. **Clear Space**: Test in an open area with at least 3m clearance in all directions
2. **Emergency Stop**: Keep the app ready to switch to manual mode or emergency stop
3. **Start Slow**: Always start with the simple forward test at low speed
4. **Monitor Robot**: Watch the robot during all tests
5. **Stop Distance**: Be prepared to stop the robot if it approaches obstacles
6. **Auto Mode**: Robot must be in AUTO mode (set via app) to respond to velocity commands

## Troubleshooting

### Robot doesn't move
- **Check robot mode**: Ensure robot is in AUTO mode via app
- **Check message_transformer**: `rosnode list` should show message_transformer nodes
- **Check cmd_vel**: Verify commands are published: `rostopic echo /cmd_vel`
- **Check UDP connection**: Motion host must be receiving UDP messages
- **Check odometry**: `rostopic hz /leg_odom` should show ~100 Hz

### Robot moves erratically
- **Lower speed**: Reduce linear and angular velocities in the script
- **Check network**: Ensure stable UDP connection to motion host
- **Check terrain**: Test on flat, even ground

### "Unable to communicate with master" error
- **Start roscore**: Run `roscore` first if not started
- **Check ROS_MASTER_URI**: Ensure ROS environment is properly configured

### message_transformer not found
- **Build package**: `cd ~/catkin_ws && catkin_make`
- **Source workspace**: `source ~/catkin_ws/devel/setup.bash`
- **Check package**: `rospack find message_transformer`

## Customizing the Test

Edit `/home/yc/catkin_ws/src/lite3_ctrl/scripts/simple_walk_test.py` to customize:

```python
# Adjust speeds (in run_simple_forward_test or run_basic_test methods)
self.walk_forward(speed=0.2, duration=5.0)  # speed in m/s, duration in seconds

# Add new movements
def custom_pattern(self):
    self.walk_forward(speed=0.3, duration=2.0)
    self.turn_left(angular_speed=0.3, duration=1.0)
    self.walk_forward(speed=0.3, duration=2.0)
    self.stop(duration=1.0)
```

## Velocity Limits

Recommended safe limits for Lite3:
- **Linear X (forward/back)**: -0.5 to 0.5 m/s (max ~1.0 m/s)
- **Linear Y (strafe)**: -0.3 to 0.3 m/s (max ~0.5 m/s)
- **Angular Z (yaw)**: -0.5 to 0.5 rad/s (max ~1.0 rad/s)

Start with conservative values and increase gradually after confirming stable operation.

## Integration with Autonomous Navigation

Once basic walking is verified, you can integrate with the EGO planner:

```bash
# Launch full autonomous navigation with Lite3
roslaunch lite3_ctrl lite3_autonomous_nav.launch
```

This will use the VK180 sensor for obstacle avoidance and autonomous navigation.

## Files

- `scripts/simple_walk_test.py`: Main test script
- `launch/lite3_walk_test.launch`: Launch file with message_transformer
- `launch/lite3_walk_test_standalone.launch`: Launch file for script only

## Next Steps

After verifying basic locomotion:
1. Test with obstacles using VK180 point cloud
2. Test autonomous navigation with EGO planner
3. Tune velocity controller parameters for smoother motion
4. Add custom walking patterns for your application

## Support

For issues with:
- **Lite3 hardware/SDK**: Contact Deep Robotics support
- **ROS integration**: Check Lite3_ROS documentation
- **Navigation/planning**: See lite3_ctrl README.md

## License

MIT
