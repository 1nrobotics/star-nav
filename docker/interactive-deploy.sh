#!/bin/bash

# Star Nav Interactive Docker Deployment
# Starts an interactive development container

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚁 Star Nav - Interactive Development Container${NC}"

# Enable X11 forwarding for GUI applications
xhost +local:docker 2>/dev/null || true

# Set default environment variables
export IMAGE_TAG=${IMAGE_TAG:-local}
export IMAGE_NAME=${IMAGE_NAME:-1nrobotics/star-nav}
export DRONE_ID=${DRONE_ID:-1}
export VEHICLE_ID=${VEHICLE_ID:-1}
export DISPLAY=${DISPLAY}

# Check if image exists, if not build it
if ! docker images | grep -q "${IMAGE_NAME}"; then
    echo "Image not found. Building..."
    cd /home/yingcai/src/star-nav
    docker build -t ${IMAGE_NAME}:${IMAGE_TAG} .
fi

# Start interactive container
echo "Starting interactive development container..."
docker run -it --rm \
    --name star_nav_interactive \
    --network host \
    --privileged \
    -v /home/yingcai/src/star-nav/px4ctrl:/catkin_ws/src/px4ctrl:rw \
    -v /home/yingcai/src/star-nav/planner:/catkin_ws/src/planner:rw \
    -v /home/yingcai/src/star-nav/shiviz_px4_nav:/catkin_ws/src/shiviz_px4_nav:rw \
    -v /home/yingcai/src/star-nav/vk180-ros:/catkin_ws/src/vk180-ros:rw \
    -v /home/yingcai/src/star-nav/utils:/catkin_ws/src/utils:rw \
    -v /home/yingcai/src/star-nav/realsense-ros:/catkin_ws/src/realsense-ros:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e DISPLAY="${DISPLAY}" \
    -e ROS_MASTER_URI="http://localhost:11311" \
    -e ROS_HOSTNAME="localhost" \
    -e ROS_IP="localhost" \
    -e PX4_SIM_HOST="localhost" \
    -e PX4_SIM_MODEL="iris" \
    -e VEHICLE_ID="${VEHICLE_ID}" \
    ${IMAGE_NAME}:${IMAGE_TAG} \
    bash

echo -e "${GREEN}✅ Interactive session ended${NC}"
