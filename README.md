# Star Nav

Star Nav is a ROS Noetic workspace for quadrotor planning, simulation, PX4
control, and optional VK180-based onboard perception.

This open-source branch is organized around a simple rule:

- use simulation first
- document real hardware second

## Status

- Simulation is the recommended public entrypoint.
- Real-hardware support remains available, but it depends on proprietary
  VK180/Vilota tooling.
- The repository contains mixed licensing inherited from upstream components.
  Start with [docs/LICENSES.md](docs/LICENSES.md).

## Quick Start

### Recommended: Docker simulation workflow

```bash
cd /path/to/star-nav
ENABLE_VK180_SDK=false REBUILD_IMAGE=true ./docker/local-deploy.sh
```

Terminal 1:

```bash
./scripts/run_sim_exploration.sh
```

Terminal 2:

```bash
./scripts/trigger_exploration.sh
```

To customize the search area:

```bash
./scripts/run_sim_exploration.sh exploration_width:=12 exploration_height:=12 line_spacing:=4
```

### Native ROS Noetic workflow

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths . --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release -DENABLE_VK180_SDK=OFF
source devel/setup.bash
roslaunch vk180_ros exploration_sim.launch use_rviz:=true
rostopic pub -1 /exploration/trigger geometry_msgs/PoseStamped "{}"
```

## Documentation

- [Simulation Guide](docs/SIMULATION.md)
- [Hardware Guide](docs/HARDWARE.md)
- [Licensing Notes](docs/LICENSES.md)
- [Contributing](CONTRIBUTING.md)
- [Open-Source Branch Plan](docs/OPEN_SOURCE_PLAN.md)

## Repository Layout

- `planner/`: EGO-Planner-based planning stack and exploration manager
- `px4ctrl/`: PX4 controller integration
- `uav_simulator/`: simulation, mapping, and local sensing components
- `vk180-ros/`: VK180 launch files and hardware bridge code
- `utils/`: shared ROS messages, plugins, and utilities
- `docker/`: Docker-based developer and release workflows
- `scripts/`: helper scripts for local build and deployment

## Main Entry Points

### Simulation

```bash
roslaunch vk180_ros exploration_sim.launch use_rviz:=true
```

### Real exploration flight

```bash
roslaunch vk180_ros vk180_exploration_flight.launch use_rviz:=false
```

## Notes for Public Users

- The Docker simulation path can now be built without the VK180 SDK.
- The hardware bridge binaries in `vk180-ros` are optional and only build when
  VK180 SDK support is enabled.
- Some subpackage READMEs still reflect earlier internal workflows; when in
  doubt, prefer the root docs in this branch.
