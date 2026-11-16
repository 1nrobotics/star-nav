# Star Nav simulation image.
# The base image is multi-arch (amd64 / arm64), so the same file builds on
# Linux, Apple Silicon and Windows (WSL2) hosts.
ARG BASE_IMAGE=ros:noetic-ros-core-focal
FROM ${BASE_IMAGE}

LABEL maintainer="1nrobotics@github"
LABEL description="Star Nav - EGO-Planner based waypoint exploration with a headless simulation"

ENV ROS_DISTRO=noetic
ENV DEBIAN_FRONTEND=noninteractive
ENV DISABLE_ROS1_EOL_WARNINGS=1

# Build tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    vim \
    git \
    tmux \
    build-essential \
    python3-rosdep \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# ROS and system dependencies (rosdep below fills in anything missing)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-eigen-conversions \
    ros-${ROS_DISTRO}-cmake-modules \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    libeigen3-dev \
    libpcl-dev \
    libarmadillo-dev \
    qtbase5-dev \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

WORKDIR /catkin_ws/src/
COPY . /catkin_ws/src/

WORKDIR /catkin_ws/
RUN source /opt/ros/noetic/setup.bash \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/* \
    && catkin_make -DCMAKE_BUILD_TYPE=Release

COPY docker/.bashrc /root/.bashrc_custom
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    cat /root/.bashrc_custom >> /root/.bashrc

WORKDIR /catkin_ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
