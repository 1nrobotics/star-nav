#!/bin/bash

# Simple VK180 EGO Planner Launch Script
# For quick testing without tmux complexity

set -e

WORKSPACE_PATH="/home/yc/catkin_ws"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🚀 Simple VK180 EGO Planner Launch${NC}"
echo -e "${BLUE}==================================${NC}"

# Setup environment
cd $WORKSPACE_PATH
source devel/setup.bash

echo -e "${GREEN}[INFO]${NC} Starting VK180 EGO Planner Integration..."
echo -e "${GREEN}[INFO]${NC} RViz will open for visualization"
echo -e "${GREEN}[INFO]${NC} Use 2D Nav Goal in RViz to set navigation targets"
echo ""

# Launch the integrated system
roslaunch vk180_ros vk180_ego_planner_integration.launch

echo -e "${GREEN}[INFO]${NC} Launch completed."