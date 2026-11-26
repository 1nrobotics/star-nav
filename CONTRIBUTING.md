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

- Update user-facing docs when launch commands or prerequisites change,
  including the project page in `site/` (see `site/README.md`).
- Keep package license metadata accurate when moving or importing code.
- Run the tests on your machine and push only when they pass:

  ```bash
  ./scripts/run_local_tests.sh
  ```

  It runs, on a native ROS Noetic workspace, `bash -n` on the shell scripts,
  `catkin_make`, the planner unit tests, `pytest` in `mujoco_sim/`, and the
  end-to-end missions: `sim_smoke.test`, `mujoco_smoke.test` (depth camera
  and lidar), and `lite3_smoke.test` and `t1_smoke.test` with each local
  planner when the robots are downloaded. It prints a summary and exits
  non-zero when a step fails. Steps can be picked by name
  (`./scripts/run_local_tests.sh lint pytest mujoco`); the script's header
  lists them.
- Without a native ROS installation, the Docker variants cover the quadrotor:
  `./scripts/run_smoke_test.sh` and `./scripts/run_mujoco_smoke_test.sh`.

No test runs on GitHub when you push or open a pull request. The `sim-smoke`
workflow (`.github/workflows/sim-smoke.yml`, the Docker variants above) can
only be started by hand from the Actions tab. The project page is deployed
when `site/` changes on `main`.

## Style

- Use the repository `.editorconfig` defaults.
- Use `.clang-format` for C++ changes when reformatting touched code.
- Avoid drive-by reformatting in unrelated files.

## Licensing

By contributing, you agree that your changes are distributed under the
repository's published licensing terms. See
[docs/LICENSES.md](docs/LICENSES.md) for the mixed
license structure of this repository.
