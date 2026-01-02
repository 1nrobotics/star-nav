#!/bin/bash

# Script to set up the Star Nav project on Jetson without Docker
# Based on the Dockerfile instructions

set -e

echo "Setting up environment variables..."
export ROS_DISTRO=noetic
export DEBIAN_FRONTEND=noninteractive

echo "Updating package list..."
sudo apt-get update

echo "Installing bootstrap tools and essential packages..."
sudo apt-get install --no-install-recommends -y \
    vim \
    wget \
    curl \
    git \
    tmux \
    iproute2 \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    python3-catkin-tools \
    python3-pip \
    xmlstarlet

echo "Bootstrapping rosdep..."
sudo rosdep init || echo "rosdep already initialized"
rosdep update --rosdistro $ROS_DISTRO

echo "Installing additional ROS packages..."
sudo apt-get install -y --no-install-recommends \
    ros-noetic-ros-base \
    ros-noetic-mavlink \
    ros-noetic-mavros \
    ros-noetic-mavros-msgs \
    ros-noetic-mavros-extras \
    ros-noetic-geometry-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-eigen-conversions \
    ros-noetic-cmake-modules \
    ros-noetic-rviz \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-roslint \
    ros-noetic-xacro \
    ros-noetic-ddynamic-reconfigure

echo "Installing system dependencies..."
sudo apt-get install -y --no-install-recommends \
    libeigen3-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    libyaml-cpp0.6

echo "Installing helper tools for Vilota installer..."
sudo apt-get install -y --no-install-recommends \
    python3-requests \
    software-properties-common \
    sudo \
    iputils-ping \
    apt-transport-https \
    ca-certificates \
    gnupg

echo "Installing eCAL, capnp and dependencies with Vilota installer..."
# Note: This may require credentials. Adjust if needed.
python3 docker/tools/VilotaSoftwareInstaller_v2.0-bba1a93f.py -s -u deployment -p vilota-dep --no-update-script

echo "Installing eCAL Python bindings..."
sudo apt-get install -y --no-install-recommends python3-ecal5 || echo "Warning: python3-ecal5 package not available"

echo "Verifying eCAL and pycapnp installation..."
python3 -c "import ecal; print('eCAL imported successfully')" || echo "Warning: eCAL import failed"
python3 -c "import capnp; print(f'pycapnp {capnp.__version__} imported successfully')" || echo "Warning: pycapnp import failed"

echo "Setting up udev rules for Movidius..."
sudo mkdir -p /etc/udev/rules.d
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules > /dev/null

echo "Installing MAVROS geographic datasets..."
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh

echo "Copying eCAL configuration..."
sudo mkdir -p /etc/ecal
sudo cp docker/tools/ecal.ini /etc/ecal/ecal.ini

# echo "Building the workspace..."
# cd /home/nvidia/src/star-nav
# source /opt/ros/noetic/setup.bash
# rosdep install --from-paths src --ignore-src -r -y
# catkin_make -DCMAKE_BUILD_TYPE=Release

# echo "Setting up bashrc..."
# echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# echo "source /home/nvidia/src/star-nav/devel/setup.bash" >> ~/.bashrc

echo "Setup complete. Source your bashrc or restart terminal."