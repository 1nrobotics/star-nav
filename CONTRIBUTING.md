# Contributing

Thanks for helping improve Star Nav.

## Scope

`main` holds the simulation and planning stack and stays hardware-independent.
Sensor-specific integration lives on `hardware/*` branches (see
[docs/HARDWARE.md](docs/HARDWARE.md)).

Please prioritize changes that keep the simulation workflow reproducible and
easy to review.

## Development Setup

### Recommended: simulation-first Docker workflow

```bash
cd /path/to/star-nav
./scripts/run_smoke_test.sh          # headless end-to-end test, any Docker host
./scripts/run_mujoco_smoke_test.sh   # same, on the MuJoCo simulator
./docker/local-deploy.sh             # Linux dev container with RViz
./scripts/run_sim_exploration.sh     # SIM=mujoco for the MuJoCo simulator
```

### Native ROS Noetic workflow

The repository is the `src` folder of a catkin workspace:

```bash
git clone https://github.com/1nrobotics/star-nav.git ~/catkin_ws/src
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch star_nav_bringup exploration_sim.launch use_rviz:=true
```

## Before Opening a PR

- Update user-facing docs when launch commands or prerequisites change.
- Keep package license metadata accurate when moving or importing code.
- Run lightweight checks that match your change:
  - `bash -n` for shell scripts
  - `./scripts/run_smoke_test.sh` (or `rostest star_nav_bringup sim_smoke.test`)
    when planner, launch or Docker behavior changes; CI runs it on every PR
  - `pytest` in `mujoco_sim/` and `./scripts/run_mujoco_smoke_test.sh` when
    the MuJoCo simulator, the bridge or its protocol changes
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
