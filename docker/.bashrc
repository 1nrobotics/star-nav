# ~/.bashrc: executed by bash(1) for non-login shells.

# ROS environment setup
export ROS_DISTRO="noetic"
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# PX4 and MAVROS configuration
export PX4_SIM_HOST=${PX4_SIM_HOST:-localhost}
export PX4_SIM_MODEL=${PX4_SIM_MODEL:-iris}

# ROS networking (can be overridden by environment variables)
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}
export ROS_IP=${ROS_IP:-localhost}

# Convenience aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'
alias grep='grep --color=auto'

# ROS aliases
alias rosenv='env | grep ROS'
alias catkin_build_release='catkin build -DCMAKE_BUILD_TYPE=Release'
alias catkin_clean_all='catkin clean --all'

# PX4 Control aliases
alias ctrl_launch='roslaunch px4ctrl run_ctrl.launch'
alias thrust_cal='roslaunch px4ctrl thrust_calibrate.launch'

# PX4 specific aliases
alias px4_arm='rosservice call /mavros/cmd/arming "value: true"'
alias px4_disarm='rosservice call /mavros/cmd/arming "value: false"'
alias px4_mode_offboard='rosservice call /mavros/set_mode "base_mode: 0 custom_mode: OFFBOARD"'
alias px4_mode_manual='rosservice call /mavros/set_mode "base_mode: 0 custom_mode: MANUAL"'

# Monitoring aliases  
alias planner_status='rostopic echo /drone_0_ego_planner_node/planning/exec_state'
alias planner_cmd='rostopic echo /drone_0_planning/pos_cmd'
alias px4_state='rostopic echo /mavros/state'
alias px4_odom='rostopic echo /mavros/local_position/odom'
alias px4_cmd='rostopic echo /mavros/setpoint_raw/attitude'

# EGO Planner simulation aliases
alias planner_single_sim='roslaunch ego_planner single_run_in_sim.launch'
alias exploration_sim='roslaunch star_nav_bringup exploration_sim.launch'
alias exploration_sim_rviz='roslaunch star_nav_bringup exploration_sim.launch use_rviz:=true'
alias exploration_rviz='rosrun rviz rviz -d $(rospack find star_nav_bringup)/config/star_nav.rviz'
alias exploration_smoke_test='rostest star_nav_bringup sim_smoke.test'
alias exploration_trigger='rostopic pub /exploration/trigger geometry_msgs/PoseStamped "{}"'


# Development helpers
alias kill_ros='killall -9 rosmaster roscore; tmux kill-server 2>/dev/null || true'
alias restart_px4ctrl='rosnode kill px4ctrl && sleep 2 && ctrl_launch'

# tmux convenience
alias tmux_nav='tmux new-session -d -s nav -c /catkin_ws'

# Color support
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
fi

# Enable programmable completion features
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

# Welcome message
echo "Star Nav Docker Environment"
echo "   ROS Master: $ROS_MASTER_URI"
echo ""
echo "Quick commands:"
echo "  exploration_sim         - Launch the exploration simulation (headless)"
echo "  exploration_sim_rviz    - Launch the exploration simulation with RViz"
echo "  exploration_trigger     - Trigger waypoint exploration"
echo "  exploration_smoke_test  - Run the end-to-end simulation test"
echo "  planner_status          - Monitor planner execution state"
echo "  planner_cmd             - Monitor planner trajectory position command"
echo ""
