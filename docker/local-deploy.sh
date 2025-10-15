#!/bin/bash

# Star Nav Local Docker Deployment Script
# Quick script for local development deployment

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚁 Star Nav - Local Deployment${NC}"

# Enable X11 forwarding for GUI applications
xhost +local:docker 2>/dev/null || true

# Set default environment variables
export IMAGE_TAG=${IMAGE_TAG:-local}
export IMAGE_NAME=${IMAGE_NAME:-1nrobotics/star-nav}
export DRONE_ID=${DRONE_ID:-1}
export VEHICLE_ID=${VEHICLE_ID:-1}
export DISPLAY=${DISPLAY}

# Stop any running containers
echo "Stopping existing containers..."
docker stop star_nav 2>/dev/null || true
docker rm star_nav 2>/dev/null || true

# Check if image exists, if not build it
if ! docker images | grep -q "${IMAGE_NAME}"; then
    echo "Image not found. Building..."
    cd /home/yc/catkin_ws/src
    docker build -t ${IMAGE_NAME}:${IMAGE_TAG} .
fi

# Create logs directory
mkdir -p /home/yc/catkin_ws/src/logs

# Start the service
echo "Starting Star Nav service..."
docker run -d \
    --name star_nav \
    --network host \
    --privileged \
    -v /home/yc/catkin_ws/src/px4ctrl:/catkin_ws/src/px4ctrl:rw \
    -v /home/yc/catkin_ws/src/shiviz_px4_nav:/catkin_ws/src/shiviz_px4_nav:rw \
    -v /home/yc/catkin_ws/src/utils:/catkin_ws/src/utils:rw \
    -v /home/yc/catkin_ws/src/logs:/catkin_ws/logs:rw \
    -v /dev:/dev:rw \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e ROS_HOSTNAME=localhost \
    -e ROS_IP=localhost \
    -e PX4_SIM_HOST=localhost \
    -e PX4_SIM_MODEL=iris \
    -e VEHICLE_ID=${VEHICLE_ID} \
    -e DISPLAY=${DISPLAY} \
    ${IMAGE_NAME}:${IMAGE_TAG} \
    bash -c "source /catkin_ws/devel/setup.bash && roslaunch shiviz_px4_nav px4_offboard.launch"

echo ""
echo -e "${GREEN}✅ Local deployment completed!${NC}"
echo ""
echo "Container status:"
docker ps | grep star_nav

echo ""
echo "Useful commands:"
echo "  docker logs -f star_nav         # View logs"
echo "  docker exec -it star_nav bash   # Access container"
echo "  docker stop star_nav            # Stop service"
echo "  docker rm star_nav              # Remove container"
