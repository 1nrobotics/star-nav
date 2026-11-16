# Simulation Guide

This is the recommended way to evaluate and develop Star Nav.

## What You Get

The simulation flow launches:

- procedural map generation
- the EGO planner stack
- a simulated quadrotor with a depth camera
- the exploration manager
- optional RViz visualization

No flight controller or sensor hardware is required.

Two simulators are available behind the same ROS topics:

| | Kinematic (`exploration_sim.launch`) | MuJoCo (`exploration_mujoco.launch`) |
| --- | --- | --- |
| Vehicle | follows the planner command exactly | rigid-body dynamics, motor lag, SE(3) flight controller |
| Depth camera | CPU point-cloud rendering | OpenGL rendering (EGL or OSMesa) |
| Collisions | not detected | counted on `/mujoco/contacts` |
| Visualisation | RViz | browser (http://localhost:8080) and RViz |
| Clock | wall time | simulation time (`/clock` from the bridge) |
| Requirements | ROS Noetic | ROS Noetic + Python >= 3.11 (or Docker) |

The kinematic simulator is the quickest check of planner logic; MuJoCo shows
how well the plan can actually be flown.

## Headless Test (any Docker host)

Runs on Linux, macOS (including Apple Silicon) and Windows with WSL2:

```bash
cd /path/to/star-nav
./scripts/run_smoke_test.sh
```

Equivalent to `docker compose run --rm --build test`. The test triggers the
default 9 m x 9 m mission and checks that:

- the exploration FSM reaches `FINISH`
- all 12 waypoints are reached without a waypoint timeout
- the flown path covers at least 80% of the exploration area

It exits non-zero on failure, so it can be used in CI and before every push.

## MuJoCo Simulator

Headless test in Docker (any OS):

```bash
./scripts/run_mujoco_smoke_test.sh
```

It runs the same mission as the kinematic test and additionally fails if the
quadrotor touches an obstacle.

Interactive, in Docker (Linux host, after `./docker/local-deploy.sh`):

```bash
SIM=mujoco ./scripts/run_sim_exploration.sh
```

This starts the simulator container and the planning stack; open
http://localhost:8080 for the 3D view and trigger exploration as below.

Native (Ubuntu 20.04 with ROS Noetic, plus any Python >= 3.11):

```bash
./scripts/setup_mujoco_env.sh
roslaunch star_nav_bringup exploration_mujoco.launch use_rviz:=true
rostest star_nav_bringup mujoco_smoke.test          # headless test
```

Details, options and the bridge protocol: [mujoco_sim/README.md](../mujoco_sim/README.md).

## Interactive Workflow: Docker (Linux host)

### 1. Build and start the development container

```bash
./docker/local-deploy.sh
```

Use `REBUILD_IMAGE=true ./docker/local-deploy.sh` to rebuild from scratch.
Source directories are mounted into the container, so edits on the host are
visible inside it; rebuild with `catkin_make` inside the container after
changing C++ code.

### 2. Launch the simulation

Keep this command running in its own terminal:

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

Open a second terminal in the repository root and run:

```bash
./scripts/trigger_exploration.sh
```

or press **Start Exploration** in the RViz GCS panel.

### What RViz shows

- coloured point cloud: the generated forest map (pillars and rings, colour =
  height) and the planner's inflated occupancy map
- blue line: the path the vehicle has flown
- red line: the optimized trajectory currently being executed
- axes marker: the vehicle pose
- GCS panel: start button, exploration area and current position

## Native ROS Noetic Workflow

Useful on Ubuntu 20.04 without Docker. The repository is the `src` folder of
a catkin workspace:

```bash
git clone https://github.com/1nrobotics/star-nav.git ~/catkin_ws/src
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch star_nav_bringup exploration_sim.launch use_rviz:=true
rostopic pub -1 /exploration/trigger geometry_msgs/PoseStamped "{}"
```

Run the end-to-end test natively with:

```bash
rostest star_nav_bringup sim_smoke.test
```

If `python3` on your `PATH` is not the system Python (for example a conda
environment), the test node cannot import `rospy`; deactivate that environment
first.

## Useful Topics

```bash
rostopic echo /exploration/state
rostopic echo /drone_0_planning/pos_cmd
rostopic echo /move_base_simple/goal
```

## Troubleshooting

### RViz does not appear

- Confirm X11 access is available on the host (`xhost +local:docker`).
- Try `USE_RVIZ=false` first to validate the rest of the stack.
- On macOS or Windows, use the headless test; X11 forwarding is not set up
  for those hosts.

### Exploration does not start

- Confirm the simulation launch is still running.
- Publish the trigger once with `./scripts/trigger_exploration.sh`.
- Check `rostopic echo /exploration/state` for FSM progress.
