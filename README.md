## Release image (developer only)
docker stop star_nav && docker rm star_nav && docker rmi 1nrobotics/star-nav:release
ENABLE_REMOTE_VIZ=false ./docker/release-deploy.sh

docker tag 1nrobotics/star-nav:release vcmman/star-nav:release-jetson-v0.2
docker login
docker push vcmman/star-nav:release-jetson-v0.2

## Sim test
roslaunch ego_planner single_run_in_sim.launch
roslaunch ego_planner exploration_sim.launch
rostopic pub /exploration/trigger geometry_msgs/PoseStamped "{}"

## Field Test Release
# start everything
```bash
ssh nvidia@192.168.10.200
cd src/star-nav
./scripts/start_onboard.sh  # start camera and vio
./docker/release_deploy.sh  # start container, if pulled image, modify this script

docker exec -it star_nav bash
tmux
nav_launch     # tmux window 1, launch all nodes without online rviz
px4_odom       # tmux window 2, monitor local odom fused by px4
px4_cmd        # tmux window 3, monitor thrust and attitude command sent from px4ctrl to px4
planner_cmd    # tmux window 4, monitor planner pos command sent from planner to px4ctrl
```
# send a waypoint with topic or click on remote rviz
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'world'
pose:
  position:
    x: 1.0
    y: 0.0
    z: 2
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"
```

# In RViz, add these displays:
# 1. MarkerArray - Topic: /exploration/frontiers
# 2. PointCloud2 - Topic: /exploration/frontier_cells
# 3. Pose - Topic: /move_base_simple/goal (direct EGO planner control)
# 4. PointCloud2 - Topic: /vk180/depth/image_rect_raw (depth map, for real flight)

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

# Check distance to goal
rostopic echo /mavros/local_position/odom | grep -A3 "position"

## Field Test Dev
```bash
ssh nvidia@192.168.10.200
cd src/star-nav
./scripts/start_onboard.sh
./docker/local_deploy.sh

docker exec -it star_nav bash
tmux
roslaunch vk180_ros vk180_exploration_flight.launch use_rviz:=false 
ctrl-b-%
rostopic echo /mavros/local_position/odom
rostopic echo /mavros/setpoint_raw/attitude

# Trigger exploration
rostopic pub /exploration/trigger geometry_msgs/PoseStamped "{}"

# Or send manual goal (note: flight height is hardcoded to 1.0m in ego-planner)
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'world'
pose:
  position:
    x: 1.0
    y: 0.0
    z: 1.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"
    
rostopic echo /drone_0_planning/pos_cmd
```
