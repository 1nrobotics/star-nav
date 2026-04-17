# Contributing

Thanks for helping improve Star Nav.

## Scope

This repository combines a public simulation and planning stack with optional
hardware integration for the VK180 sensor and PX4-based flight systems.

Please prioritize changes that keep the simulation workflow reproducible and
easy to review.

## Development Setup

### Recommended: simulation-first Docker workflow

```bash
cd /path/to/star-nav
ENABLE_VK180_SDK=false REBUILD_IMAGE=true ./docker/local-deploy.sh
./scripts/run_sim_exploration.sh
```

### Native ROS Noetic workflow

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths . --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch vk180_ros exploration_sim.launch use_rviz:=true
```

## Before Opening a PR

- Update user-facing docs when launch commands or prerequisites change.
- Keep package license metadata accurate when moving or importing code.
- Run lightweight checks that match your change:
  - `bash -n` for shell scripts
  - a simulation launch smoke test when launch or Docker behavior changes
  - targeted catkin build if you touched package build files

## Style

- Use the repository `.editorconfig` defaults.
- Use `.clang-format` for C++ changes when reformatting touched code.
- Avoid drive-by reformatting in unrelated files.

## Licensing

By contributing, you agree that your changes are distributed under the
repository's published licensing terms. See
[docs/LICENSES.md](docs/LICENSES.md) for the mixed
license structure of this repository.
