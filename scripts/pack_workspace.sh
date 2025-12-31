#!/bin/bash

# Script to clean up the workspace for distribution
# Removes source code for specified packages, keeps necessary files for running and modifying others

set -e

WORKSPACE_DIR="/home/nvidia/catkin_ws"
PACKAGES_TO_CLEAN=("vk180-ros" "planner/exploration_manager" "rviz_gcs_plugin")

echo "Cleaning source code for specified packages..."

for pkg in "${PACKAGES_TO_CLEAN[@]}"; do
    echo "Cleaning $pkg..."
    pkg_path="$WORKSPACE_DIR/src/$pkg"
    if [ -d "$pkg_path" ]; then
        # Keep launch files, config files, scripts, package.xml
        find "$pkg_path" -name "*.launch" -o -name "*.yaml" -o -name "*.rviz" -o -name "*.ini" -o -name "*.sh" -o -name "package.xml" -o -name "docker" -o -name "scripts" | while read file; do
            echo "Keeping $file"
        done
        # Remove source files
        find "$pkg_path" -name "*.cpp" -o -name "*.h" -o -name "*.hpp" -o -name "CMakeLists.txt" | xargs rm -f
        # Remove src directory
        if [ -d "$pkg_path/src" ]; then
            rm -rf "$pkg_path/src"
        fi
        # Remove include directory
        if [ -d "$pkg_path/include" ]; then
            rm -rf "$pkg_path/include"
        fi
    else
        echo "Package $pkg not found in $pkg_path"
    fi
done

echo "Creating archive..."
cd /home/nvidia
tar -czf star-nav-release.tar.gz --exclude='catkin_ws/build' catkin_ws

echo "Archive created: /home/nvidia/star-nav-release.tar.gz"
echo "To extract: tar -xzf star-nav-release.tar.gz"