# Hardware Guide

This guide documents the real-system path after the simulation workflow is
working.

## Scope

The hardware stack integrates:

- VK180 sensor bridge
- PX4 via MAVROS
- `px4ctrl`
- EGO planner
- exploration manager

## Important Limitation

The hardware bridge depends on proprietary VK180/Vilota SDK components that are
not required for public simulation.

For public contributors, the simulation flow in
[docs/SIMULATION.md](SIMULATION.md) is the
recommended starting point.

## Onboard Preparation

Typical host preparation scripts:

```bash
./scripts/onboard_setup.sh
./scripts/start_onboard.sh
```

Jetson-specific setup helpers are available in:

- `scripts/setup_jetson.sh`
- `scripts/build_jetson.sh`
- `docker/run-jetson-release.sh`

## Local Development Image With VK180 SDK

If you have access to the proprietary dependencies, build the local image with:

```bash
ENABLE_VK180_SDK=true REBUILD_IMAGE=true ./docker/local-deploy.sh
```

## Launch the Real Exploration Stack

With the container running:

```bash
./scripts/run_real_exploration.sh
```

Equivalent direct ROS command inside the container:

```bash
source /catkin_ws/devel/setup.bash
roslaunch vk180_ros vk180_exploration_flight.launch use_rviz:=false
```

## Typical Runtime Checks

```bash
rostopic echo /mavros/local_position/odom
rostopic echo /drone_0_planning/pos_cmd
rostopic echo /mavros/setpoint_raw/attitude
rostopic echo /exploration/state
```

## Trigger Exploration

```bash
rostopic pub -1 /exploration/trigger geometry_msgs/PoseStamped "{}"
```

## Safety Notes

- Verify manual takeover and emergency-stop procedures before autonomous flight.
- Treat the simulation-tuned parameters as a starting point, not field-ready
  guarantees.
- Validate frame conventions, depth topics, and PX4 estimator health before
  issuing goals.
