# Star Nav

Star Nav is a ROS Noetic workspace for quadrotor waypoint exploration: an
EGO-Planner based local planner, a waypoint exploration manager, a lightweight
simulator, and a PX4 controller.

The `main` branch is simulation-first and hardware-independent. Integration
with specific onboard sensors lives on dedicated hardware branches (see
[Hardware Guide](docs/HARDWARE.md)).

## Status

- Simulation is the recommended entrypoint and is covered by an automated
  end-to-end test.
- The repository contains mixed licensing inherited from upstream components.
  Start with [docs/LICENSES.md](docs/LICENSES.md).

## Quick Start

### Fastest check: headless test in Docker (Linux, macOS, Windows/WSL2)

```bash
cd /path/to/star-nav
./scripts/run_smoke_test.sh
```

This builds the image, runs a full exploration mission without any GUI, and
exits non-zero if the mission fails.

### Interactive simulation with RViz (Linux host)

```bash
./docker/local-deploy.sh
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
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch star_nav_bringup exploration_sim.launch use_rviz:=true
rostopic pub -1 /exploration/trigger geometry_msgs/PoseStamped "{}"
```

## Documentation

- [Simulation Guide](docs/SIMULATION.md)
- [Hardware Guide](docs/HARDWARE.md)
- [Licensing Notes](docs/LICENSES.md)
- [Contributing](CONTRIBUTING.md)
- [Open-Source Branch Plan](docs/OPEN_SOURCE_PLAN.md)

## Repository Layout

- `star_nav_bringup/`: launch files, RViz config and the simulation smoke test
- `planner/`: EGO-Planner-based planning stack and exploration manager
- `uav_simulator/`: simulation, mapping, and local sensing components
- `px4ctrl/`: PX4 controller integration
- `rviz_gcs_plugin/`: RViz panel to trigger and monitor exploration
- `utils/`: shared ROS messages, plugins, and utilities
- `docker/`, `docker-compose.yml`: container build and development workflow
- `scripts/`: helper scripts for the Docker workflow

## Main Entry Points

```bash
roslaunch star_nav_bringup exploration_sim.launch use_rviz:=true   # simulation
rostest star_nav_bringup sim_smoke.test                            # end-to-end test
```

## Notes for Public Users

- Some subpackage READMEs still reflect earlier internal workflows; when in
  doubt, prefer the root docs in this branch.
