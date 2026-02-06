# Star Nav Lite

Open source autonomous navigation system for UAVs with exploration capabilities.

## Quick Start - Simulation

### Build and Run
```bash
# Build the Docker image
REBUILD_IMAGE=true IMAGE_TAG=local ENABLE_REMOTE_VIZ=true ./docker/local-deploy.sh

# Or without remote visualization
REBUILD_IMAGE=true ENABLE_REMOTE_VIZ=false ./docker/local-deploy.sh
```

### Run Exploration Simulation
```bash
# With RViz
roslaunch ego_planner single_run_in_sim.launch

# View with RViz
rosrun rviz rviz

# Trigger exploration
rostopic pub /exploration/trigger geometry_msgs/PoseStamped "{}"
```

## Local Build

Extract files to local folder:
```bash
cd catkin_ws
source devel/setup.bash
catkin_make
```

Run launch files or modify packages as needed.

## Release Image (Developer Only)
```bash
docker stop star_nav && docker rm star_nav && docker rmi 1nrobotics/star-nav:release
ENABLE_REMOTE_VIZ=false ./docker/release-deploy.sh

docker tag 1nrobotics/star-nav:release 1nrobotics/star-nav:release-v0.5
docker login
docker push 1nrobotics/star-nav:release-v0.5
```

## Sim Test
```bash
roslaunch ego_planner single_run_in_sim.launch
```

## Exploration Commands

```bash
# Trigger waypoint exploration
rostopic pub /exploration/trigger geometry_msgs/PoseStamped "{}"

# FSM returns to WAIT_TRIGGER
rostopic pub /exploration/trigger_once std_msgs/Bool "data: false"

# Emergency stop exploration
rostopic pub /exploration/finish std_msgs/Empty "{}"

# Monitor exploration state
rostopic echo /exploration/state

# Watch frontier detection
rostopic echo /exploration/frontiers

# Check if goals are being published
rostopic echo /move_base_simple/goal

# Monitor EGO planner status
rostopic echo /drone_0_planning/data_display
```
