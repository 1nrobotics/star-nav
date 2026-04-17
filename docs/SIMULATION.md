# Simulation Guide

This is the recommended way to evaluate the repository as an open-source user.

## What You Get

The simulation flow launches:

- procedural map generation
- the EGO planner stack
- the SO3 quadrotor simulator
- the exploration manager
- optional RViz visualization

No VK180 hardware or PX4 flight controller is required for this path.

## Recommended Workflow: Docker

### 1. Build and start the development container

```bash
cd /path/to/star-nav
ENABLE_VK180_SDK=false REBUILD_IMAGE=true ./docker/local-deploy.sh
```

`ENABLE_VK180_SDK=false` keeps the image public-friendly by skipping the
proprietary VK180 SDK during build. This is the default path for simulation.

### 2. Launch the simulation

```bash
./scripts/run_sim_exploration.sh
```

To disable RViz:

```bash
USE_RVIZ=false ./scripts/run_sim_exploration.sh
```

To override exploration area parameters:

```bash
./scripts/run_sim_exploration.sh exploration_width:=12 exploration_height:=12 line_spacing:=4
```

### 3. Trigger waypoint exploration

```bash
./scripts/trigger_exploration.sh
```

## Direct ROS Commands

If you already have a configured ROS environment inside the container:

```bash
source /catkin_ws/devel/setup.bash
roslaunch vk180_ros exploration_sim.launch use_rviz:=true
rostopic pub -1 /exploration/trigger geometry_msgs/PoseStamped "{}"
```

## Native ROS Noetic Workflow

This path is useful if you want to work without Docker.

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths . --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release -DENABLE_VK180_SDK=OFF
source devel/setup.bash
roslaunch vk180_ros exploration_sim.launch use_rviz:=true
```

## Useful Topics

```bash
rostopic echo /exploration/state
rostopic echo /drone_0_planning/pos_cmd
rostopic echo /move_base_simple/goal
```

## Troubleshooting

### The Docker build asks for VK180 / Vilota dependencies

Make sure you built with:

```bash
ENABLE_VK180_SDK=false ./docker/local-deploy.sh
```

### RViz does not appear

- Confirm X11 access is available on the host.
- Try `USE_RVIZ=false` first to validate the rest of the stack.
- If running remotely, set `ENABLE_REMOTE_VIZ=true` when starting the container.

### Exploration does not start

- Confirm the simulation launch is still running.
- Publish the trigger once with `./scripts/trigger_exploration.sh`.
- Check `rostopic echo /exploration/state` for FSM progress.
