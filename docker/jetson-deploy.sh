#!/bin/bash

# Star Nav Jetson NX Deployment Script
# Deployment for NVIDIA Jetson NX with GPU acceleration

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚁 Star Nav - Jetson NX Deployment${NC}"

# Enable X11 forwarding for GUI applications
xhost +local:docker 2>/dev/null || true

# Set default environment variables
export IMAGE_TAG=${IMAGE_TAG:-jetson}
export IMAGE_NAME=${IMAGE_NAME:-1nrobotics/star-nav}
export DRONE_ID=${DRONE_ID:-1}
export VEHICLE_ID=${VEHICLE_ID:-1}
export DISPLAY=${DISPLAY:-:0}

# Stop any running containers
echo "Stopping existing containers..."
docker stop star_nav 2>/dev/null || true
docker rm star_nav 2>/dev/null || true

# Check if image exists
if ! docker images | grep -q "${IMAGE_NAME}.*${IMAGE_TAG}"; then
    echo "Image ${IMAGE_NAME}:${IMAGE_TAG} not found. Building for Jetson..."
    
    # Build with Jetson-specific base image (or use Dockerfile.jetson)
    # Option 1: Use standard Dockerfile (works if built natively on Jetson)
    docker build -t ${IMAGE_NAME}:${IMAGE_TAG} ..
    
    # Option 2: Use Jetson-specific Dockerfile
    # docker build -f Dockerfile.jetson -t ${IMAGE_NAME}:${IMAGE_TAG} ..
fi

# Start the service with GPU support
echo "Starting Star Nav service with GPU acceleration..."
docker run -d \
    --name star_nav \
    --runtime nvidia \
    --network host \
    --privileged \
    -v /dev:/dev:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd)/px4ctrl:/catkin_ws/src/px4ctrl:rw \
    -v $(pwd)/planner:/catkin_ws/src/planner:rw \
    -v $(pwd)/shiviz_px4_nav:/catkin_ws/src/shiviz_px4_nav:rw \
    -v $(pwd)/vk180-ros:/catkin_ws/src/vk180-ros:rw \
    -v $(pwd)/utils:/catkin_ws/src/utils:rw \
    -v $(pwd)/realsense-ros:/catkin_ws/src/realsense-ros:rw \
    -e DISPLAY=${DISPLAY} \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e ROS_HOSTNAME=localhost \
    -e ROS_IP=localhost \
    -e PX4_SIM_HOST=localhost \
    -e PX4_SIM_MODEL=iris \
    -e VEHICLE_ID=${VEHICLE_ID} \
    ${IMAGE_NAME}:${IMAGE_TAG} \
    bash -c "source /catkin_ws/devel/setup.bash && roslaunch vk180_ros vk180_autonomous_flight.launch use_sim:=false"

echo ""
echo -e "${GREEN}✅ Jetson deployment completed!${NC}"
echo ""
echo "Container status:"
docker ps | grep star_nav

echo ""
echo "Useful commands:"
echo "  docker logs -f star_nav         # View logs"
echo "  docker exec -it star_nav bash   # Access container"
echo "  docker stop star_nav            # Stop service"
echo "  docker rm star_nav              # Remove container"
echo ""
echo "GPU monitoring:"
echo "  sudo tegrastats                 # Jetson system stats"
echo "  nvidia-smi                      # GPU usage (if available)"
