#!/bin/bash

# VK180 EGO Planner Integration Launch Script with TMUX
# This script launches VK180 sensors, ROS bridge, EGO planner, and visualization

set -e

# Configuration
SESSION_NAME="vk180_planner"
WORKSPACE_PATH="/home/yc/catkin_ws"
WAIT_TIME=3

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 VK180 EGO Planner Integration Launch Script${NC}"
echo -e "${BLUE}=============================================${NC}"

# Function to print colored messages
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    print_error "tmux is not installed. Please install it with: sudo apt install tmux"
    exit 1
fi

# Check if ROS workspace exists
if [ ! -d "$WORKSPACE_PATH" ]; then
    print_error "ROS workspace not found at $WORKSPACE_PATH"
    exit 1
fi

# Function to create tmux window with ROS environment
create_ros_window() {
    local window_name=$1
    local command=$2
    
    tmux new-window -t "$SESSION_NAME" -n "$window_name"
    tmux send-keys -t "$SESSION_NAME:$window_name" "cd $WORKSPACE_PATH" Enter
    tmux send-keys -t "$SESSION_NAME:$window_name" "source devel/setup.bash" Enter
    tmux send-keys -t "$SESSION_NAME:$window_name" "$command" Enter
}

# Kill existing session if it exists
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    print_warning "Killing existing tmux session: $SESSION_NAME"
    tmux kill-session -t "$SESSION_NAME"
fi

print_status "Creating new tmux session: $SESSION_NAME"

# Create main tmux session
tmux new-session -d -s "$SESSION_NAME" -n "roscore"

# Setup ROS environment for main window
tmux send-keys -t "$SESSION_NAME:roscore" "cd $WORKSPACE_PATH" Enter
tmux send-keys -t "$SESSION_NAME:roscore" "source devel/setup.bash" Enter

print_status "Starting ROS core..."
tmux send-keys -t "$SESSION_NAME:roscore" "roscore" Enter

# Wait for roscore to start
sleep $WAIT_TIME

print_status "Starting VK180 Enhanced Planner Integration..."
create_ros_window "vk180_integration" "roslaunch vk180_ros vk180_ego_planner_integration.launch"

# Wait for integration to initialize
sleep $WAIT_TIME

print_status "Starting additional debugging tools..."

# Create window for sensor extrinsics monitoring
create_ros_window "sensor_monitor" "echo 'Sensor monitoring tools ready. Commands:
- ./devel/lib/vk180_ros/sensor_extrinsics
- ./devel/lib/vk180_ros/depth_conversion_test  
- rostopic echo /vk180/odom
- rostopic echo /vk180/depth'"

# Create window for planner monitoring
create_ros_window "planner_monitor" "echo 'Planner monitoring ready. Commands:
- rostopic echo /drone_0_planning/data_display
- rostopic list | grep drone_0
- rosnode info /drone_0_ego_planner_node'"

# Create window for topic monitoring
create_ros_window "topic_monitor" "echo 'Topic monitoring ready. Commands:
- rostopic hz /vk180/depth
- rostopic hz /vk180/odom
- rqt_graph
- rqt_tf_tree'"

# Create window for navigation goals
create_ros_window "navigation" "echo 'Navigation control ready. Commands:
- Use RViz 2D Nav Goal tool to set destinations
- rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped ...'
echo 'Sample navigation goal:'
echo 'rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \"header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: world
pose:
  position:
    x: 5.0
    y: 3.0
    z: 1.5
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0\"'"

# Create logs window
create_ros_window "logs" "echo 'System logs and diagnostics ready. Commands:
- tail -f ~/.ros/log/latest/rosout.log
- rostopic echo /rosout | grep ERROR
- roswtf'"

# Return to the main integration window
tmux select-window -t "$SESSION_NAME:vk180_integration"

print_status "Setting up tmux pane layouts..."

# Split the vk180_integration window to show multiple views
tmux split-window -h -t "$SESSION_NAME:vk180_integration"
tmux send-keys -t "$SESSION_NAME:vk180_integration.1" "cd $WORKSPACE_PATH && source devel/setup.bash" Enter
tmux send-keys -t "$SESSION_NAME:vk180_integration.1" "sleep 5 && rostopic hz /grid_map/depth" Enter

print_status "✅ TMUX session '$SESSION_NAME' created successfully!"
echo ""
echo -e "${BLUE}📋 Session Layout:${NC}"
echo "  🖥️  roscore        - ROS core system"
echo "  🚁  vk180_integration - Main VK180 + EGO Planner integration"
echo "  📡  sensor_monitor - VK180 sensor debugging tools"
echo "  🗺️   planner_monitor - EGO planner debugging"
echo "  📊  topic_monitor  - ROS topic monitoring"
echo "  🎯  navigation     - Navigation goal setting"
echo "  📝  logs          - System logs and diagnostics"
echo ""
echo -e "${BLUE}🎮 Controls:${NC}"
echo "  tmux attach -t $SESSION_NAME    - Attach to session"
echo "  Ctrl+b + n                      - Next window"
echo "  Ctrl+b + p                      - Previous window"
echo "  Ctrl+b + [0-6]                  - Go to specific window"
echo "  Ctrl+b + d                      - Detach from session"
echo "  tmux kill-session -t $SESSION_NAME - Kill session"
echo ""
echo -e "${BLUE}🔍 Quick Checks:${NC}"
echo "  1. Check if VK180 is streaming: rostopic hz /grid_map/depth"
echo "  2. Check odometry: rostopic echo /grid_map/odom"
echo "  3. Check planning: rostopic echo /drone_0_planning/data_display"
echo "  4. Use RViz 2D Nav Goal to test planning"
echo ""
echo -e "${GREEN}🎯 Ready to launch! Attaching to tmux session...${NC}"

# Wait a moment for everything to initialize
sleep 2

# Attach to the session
tmux attach-session -t "$SESSION_NAME"