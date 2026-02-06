# Star-Nav Copilot Instructions

## Project Overview
Star-Nav is a ROS-based autonomous navigation and exploration system for UAVs (Unmanned Aerial Vehicles). It combines real-time perception from VK180 stereo vision sensors with PX4 flight control and EGO Planner trajectory optimization to enable autonomous drone exploration.

## Tech Stack
- **Language**: C++ (primary), Python (scripts)
- **Framework**: ROS Noetic (Robot Operating System)
- **Build System**: CMake with catkin (ROS build system)
- **Hardware Targets**: 
  - x86_64 (simulation and ground station)
  - ARM64/Jetson (onboard drone computer)
  - PX4 flight controller (Pixhawk family)
  - VK180 FPGA-based stereo vision sensor
- **Key Libraries**: Eigen3, PCL (Point Cloud Library), OpenCV
- **Containerization**: Docker (multi-architecture support)

## Repository Structure

### Core Components
1. **`/planner`** - Trajectory planning and exploration
   - `exploration_manager`: Waypoint and frontier-based autonomous exploration
   - `plan_manage`: Main trajectory planning orchestration  
   - `plan_env`: Environment representation and obstacle detection
   - `bspline_opt`: B-spline trajectory optimization
   - `path_searching`: A* and kinodynamic path search
   - `traj_utils`: Trajectory utilities and message definitions

2. **`/px4ctrl`** - Low-level flight control
   - SO3 attitude controller for quadrotors
   - MAVROS integration with PX4 autopilot
   - Thrust and attitude command generation

3. **`/vk180-ros`** - Hardware integration
   - VK180 vision sensor bridge (eCAL protocol)
   - VIO-IMU fusion with PX4 EKF2
   - Point cloud and depth image processing
   - Launch files for complete system deployment

4. **`/uav_simulator`** - Simulation environment
   - SO3 quadrotor dynamics simulator
   - Local sensing simulation (depth rendering)
   - Random map generation for testing

5. **`/utils`** - Shared utilities
   - Custom ROS message definitions (`quadrotor_msgs`)
   - Pose and transformation utilities
   - CMake helpers and catkin_simple

6. **`/rviz_gcs_plugin`** - Visualization
   - Custom RViz plugin for ground control station

## Development Workflow

### Building the Project

**Local Development (on x86):**
```bash
cd ~/catkin_ws  # or wherever the workspace is located
catkin_make     # or catkin build
source devel/setup.bash
```

**Docker Build:**
```bash
# For x86_64 platform
REBUILD_IMAGE=true IMAGE_TAG=local ./docker/local-deploy.sh

# For Jetson ARM64
./scripts/build_jetson.sh
```

**Individual Package:**
```bash
cd ~/catkin_ws
catkin build <package_name>  # or catkin_make --pkg <package_name>
```

### Running the System

**Simulation:**
```bash
# Launch complete simulation environment
roslaunch vk180_ros exploration_sim.launch use_rviz:=true

# In another terminal, trigger exploration
rostopic pub /exploration/trigger geometry_msgs/PoseStamped "{}"
```

**Real Hardware:**
```bash
# On drone (Jetson)
cd src/star-nav
./scripts/start_onboard.sh  # Start camera and VIO
./docker/release-deploy.sh  # Start ROS container

# Inside container
roslaunch vk180_ros vk180_exploration_flight.launch
```

### Testing
- No formal unit test framework currently exists
- Testing is primarily done through simulation and hardware flight tests
- Use `exploration_sim.launch` for integration testing before hardware deployment

## Coding Standards

### C++ Conventions
- Follow ROS C++ style guide conventions
- Use Eigen for linear algebra (not raw arrays)
- Header guards: `#ifndef PACKAGE_NAME_FILE_NAME_H` format
- Class names: PascalCase (e.g., `WaypointExploration`)
- Function names: camelCase (e.g., `planNextWaypoint`)
- Member variables: snake_case with trailing underscore (e.g., `max_velocity_`)
- Constants: UPPER_SNAKE_CASE or k-prefix (e.g., `kMaxVelocity`)

### ROS Conventions
- Package names: lowercase with underscores (e.g., `exploration_manager`)
- Topic names: lowercase with slashes (e.g., `/drone_0_planning/pos_cmd`)
- Node names: match the package or be descriptive (e.g., `exploration_node`)
- Launch files: descriptive names ending in `.launch`
- Use `package.xml` format 2 for dependencies

### File Organization
- Headers in `include/<package_name>/`
- Source files in `src/`
- Launch files in `launch/`
- Config files in `config/`
- Each ROS package has its own `CMakeLists.txt` and `package.xml`

## Key Architectural Patterns

### ROS Message Passing
- Use `ros::NodeHandle` for topic subscriptions/publications
- Prefer `sensor_msgs::PointCloud2` for point clouds
- Use `nav_msgs::Odometry` for pose and velocity feedback
- Custom messages defined in `utils/quadrotor_msgs`

### State Machines
- Exploration uses FSM pattern (see `exploration_manager/waypoint_exploration.cpp`)
- States: INIT → WAIT_TRIGGER → PLAN → EXEC → FINISH
- State transitions logged with ROS_INFO/WARN

### Coordinate Frames
- **World frame** (`world`): Fixed reference frame, typically ENU (East-North-Up)
- **Body frame**: Forward-Right-Down (FRD) for drone body
- **Camera frame**: Optical conventions (z forward, x right, y down)
- Use `tf2` for frame transformations when needed

### Planner Integration
- EGO planner subscribes to `/move_base_simple/goal` (geometry_msgs::PoseStamped)
- Outputs trajectory as `quadrotor_msgs::PositionCommand` to `/drone_0_planning/pos_cmd`
- Exploration manager calls EGO planner by publishing goals
- PX4 controller subscribes to position commands and generates thrust/attitude

## Common Development Tasks

### Adding a New ROS Node
1. Create source file in `src/` directory of appropriate package
2. Add executable in `CMakeLists.txt`:
   ```cmake
   add_executable(node_name src/node_name.cpp)
   target_link_libraries(node_name ${catkin_LIBRARIES})
   ```
3. Create launch file in `launch/` directory
4. Document parameters and topics in package README

### Modifying Planner Parameters
- Parameters are set in launch files (e.g., `vk180_exploration.launch`)
- Key namespaces: `manager/`, `fsm/`, `exploration/`
- Test in simulation before hardware deployment
- Document parameter changes in launch file comments

### Working with Docker
- Dockerfiles support both x86 and ARM64
- Use `docker/local-deploy.sh` for development
- Use `docker/release-deploy.sh` for production builds
- Container mounts workspace for live code editing

## Things to Avoid

### Anti-patterns
- **Don't** use blocking calls in ROS callbacks (use timers or async)
- **Don't** hardcode frame names; use parameters or tf lookups
- **Don't** ignore ROS_WARN or ROS_ERROR messages in logs
- **Don't** commit compiled binaries or build artifacts (build/, devel/, logs/)
- **Don't** use `sleep()` for synchronization; use ROS rate or timers

### Safety Considerations
- Always validate waypoints are within safe boundaries
- Check for valid odometry before commanding motion
- Respect max velocity and acceleration limits
- Test extensively in simulation before hardware deployment
- Keep emergency RC transmitter ready during flights

### Build System
- Don't mix catkin_make and catkin build in the same workspace
- Don't modify `CMakeLists.txt` in workspace root (it's a symlink)
- Don't add dependencies without updating `package.xml`

## Exploration Modes

### Waypoint Exploration (Default)
- Systematic lawnmower pattern coverage
- Parameters: `center_x/y/z`, `width`, `height`, `line_spacing`
- Best for structured environments and predictable coverage
- Trigger: `rostopic pub /exploration/trigger geometry_msgs/PoseStamped "{}"`

### Frontier Exploration
- Dynamic boundary detection and navigation
- Adapts to environment layout
- Best for unknown or irregular spaces
- Enable with: `<param name="fsm/exploration_mode" value="frontier"/>`

## Useful Commands

### ROS Operations
```bash
# Source workspace
source devel/setup.bash

# List available launch files
roslaunch <package_name> <tab><tab>

# Monitor topics
rostopic list
rostopic hz <topic_name>
rostopic echo <topic_name>

# Check node graph
rosnode list
rosnode info <node_name>

# Visualize with RViz
rosrun rviz rviz -d <config_file>.rviz
```

### Debugging
```bash
# Check ROS master
rostopic list

# View node output
rosnode info /exploration_node

# Record data for analysis
rosbag record -a  # Record all topics
rosbag play <bag_file>

# Monitor system resources
htop
nvidia-smi  # On Jetson for GPU usage
```

### Git Workflow
```bash
# This is a catkin workspace, not just a git repo
git status
git diff
git add <files>
git commit -m "descriptive message"
git push origin <branch>
```

## Hardware-Specific Notes

### VK180 Sensor
- Provides VIO odometry at 30-60 Hz
- Outputs point clouds at 10-30 Hz
- Uses eCAL protocol for communication
- Bridge code in `vk180-ros/src/`

### PX4 Integration
- MAVROS provides ROS interface to PX4
- Fuses VIO with IMU using EKF2
- Odometry published to `/mavros/local_position/odom`
- Commands sent via `/mavros/setpoint_raw/attitude`

### Jetson Platform
- ARM64 architecture (aarch64)
- CUDA-capable for GPU acceleration
- Limited compute compared to x86
- Use Docker for deployment consistency

## External Resources
- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [EGO Planner GitHub](https://github.com/ZJU-FAST-Lab/ego-planner)
- [PX4 Autopilot Docs](https://docs.px4.io/)
- [MAVROS Wiki](http://wiki.ros.org/mavros)
- [Catkin Build System](https://catkin-tools.readthedocs.io/)

## Tips for Copilot
- When modifying planner code, always consider thread safety and ROS callback contexts
- Check existing patterns in the codebase before introducing new conventions
- Test planner changes in simulation (`exploration_sim.launch`) first
- When adding dependencies, update both `CMakeLists.txt` and `package.xml`
- Use ROS logging (`ROS_INFO`, `ROS_WARN`, `ROS_ERROR`) instead of `std::cout`
- Remember that this runs on real hardware - safety and robustness are critical
