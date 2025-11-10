## Field Test 
```bash
ssh nvidia@192.168.10.200
cd src/star-nav
./scripts/start_onboard.sh
./docker/local_deploy.sh

docker exec -it star_nav bash
tmux
roslaunch vk180_ros vk180_autonomous_flight.launch use_rviz:=false 
ctrl-b-%
rostopic echo /mavros/local_position/odom
rostopic echo /mavros/setpoint_raw/attitude

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
    
rostopic echo /drone_0_planning/pos_cmd
```
