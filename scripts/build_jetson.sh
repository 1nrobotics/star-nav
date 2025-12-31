#!/bin/bash

# Build script for Star Nav project on Jetson
# Creates a new catkin workspace, copies src, installs deps, and builds

set -e

WORKSPACE_DIR="/home/nvidia/catkin_ws"

echo "Creating new catkin workspace at $WORKSPACE_DIR..."
mkdir -p $WORKSPACE_DIR/src

echo "Copying source code from star-nav to workspace..."
# Copy only the ROS package directories
cp -r /home/nvidia/src/star-nav/planner $WORKSPACE_DIR/src/
cp -r /home/nvidia/src/star-nav/px4ctrl $WORKSPACE_DIR/src/
cp -r /home/nvidia/src/star-nav/rviz_gcs_plugin $WORKSPACE_DIR/src/
cp -r /home/nvidia/src/star-nav/uav_simulator $WORKSPACE_DIR/src/
cp -r /home/nvidia/src/star-nav/utils $WORKSPACE_DIR/src/
cp -r /home/nvidia/src/star-nav/vk180-ros $WORKSPACE_DIR/src/
cp -r /home/nvidia/src/star-nav/scripts $WORKSPACE_DIR/src/
cp -r /home/nvidia/src/star-nav/docker $WORKSPACE_DIR/src/
# Copy root CMakeLists.txt and other files if needed
cp /home/nvidia/src/star-nav/CMakeLists.txt $WORKSPACE_DIR/src/

echo "Sourcing ROS setup..."
source /opt/ros/noetic/setup.bash

echo "Installing ROS dependencies..."
cd $WORKSPACE_DIR
rosdep install --from-paths src --ignore-src -r -y

echo "Building the workspace..."
catkin_make -DCMAKE_BUILD_TYPE=Release install

echo "Build complete. Workspace at $WORKSPACE_DIR"
echo "To use: source $WORKSPACE_DIR/devel/setup.bash"