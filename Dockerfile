# Multi-architecture support: works for x86_64 and ARM64 (Jetson)
ARG BASE_IMAGE=ros:noetic-ros-core-focal
FROM ${BASE_IMAGE}

LABEL maintainer="1nrobotics@github"
LABEL description="Docker image for Star Nav Lite - Simplified ROS Navigation System for open source"

# Set environment variables
ENV ROS_DISTRO=noetic
ENV DEBIAN_FRONTEND=noninteractive

# Install bootstrap tools and essential packages
RUN apt-get update && apt-get install --no-install-recommends -y \
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
    xmlstarlet \
    && rm -rf /var/lib/apt/lists/*

# Bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# Install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Install MAVROS and related dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-mavlink \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS packages for navigation and control
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-eigen-conversions \
    ros-${ROS_DISTRO}-cmake-modules \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-roslint \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ddynamic-reconfigure \
    && rm -rf /var/lib/apt/lists/*

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    libyaml-cpp0.6 \
    && rm -rf /var/lib/apt/lists/*

# Note: eCAL and Vilota-specific dependencies removed for open source version

# Install MAVROS geographic datasets
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && bash ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"]

# Create workspace and copy all source code
WORKDIR /catkin_ws/src/
COPY . /catkin_ws/src/

# Initialize and build the workspace
WORKDIR /catkin_ws/
RUN source /opt/ros/noetic/setup.bash \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && catkin_make -DCMAKE_BUILD_TYPE=Release

# Build argument to control source code cleanup (default: keep source for development)
# Declared here AFTER build to allow layer caching regardless of build arg value
ARG RELEASE_BUILD=false

# Clean up source code after build (only for release builds)
# Keep: launch files, config files, scripts, and compiled binaries
# Remove: source code (.cpp, .h, CMakeLists.txt, package.xml build artifacts)
RUN if [ "$RELEASE_BUILD" = "true" ]; then \
        echo "=== Building RELEASE image - removing source code ===" && \
        cd /catkin_ws/src && \
        # Keep launch files \
        find . -name "*.launch" -o -name "*.xml" | grep -E "(launch|urdf|xacro|plugin_description)" > /tmp/keep_files.txt && \
        # Keep config files \
        find . -name "*.yaml" -o -name "*.rviz" -o -name "*.ini" -o -name "*.mesh" >> /tmp/keep_files.txt && \
        # Keep Python scripts \
        find . -name "*.py" -path "*/scripts/*" >> /tmp/keep_files.txt && \
        # Keep shell scripts \
        find . -name "*.sh" >> /tmp/keep_files.txt && \
        # Keep package.xml for roslaunch to work \
        find . -name "package.xml" >> /tmp/keep_files.txt && \
        # Keep msg/srv/action definitions (might be needed at runtime) \
        find . -path "*/msg/*" -o -path "*/srv/*" -o -path "*/action/*" >> /tmp/keep_files.txt && \
        # Create a temporary directory with only necessary files \
        mkdir -p /tmp/catkin_ws_minimal/src && \
        cd /catkin_ws/src && \
        # Copy directory structure first \
        find . -type d -exec mkdir -p /tmp/catkin_ws_minimal/src/{} \; && \
        # Copy only the files we want to keep \
        cat /tmp/keep_files.txt | while read file; do \
            if [ -f "$file" ]; then \
                cp --parents "$file" /tmp/catkin_ws_minimal/src/ || true; \
            fi; \
        done && \
        # Remove old source directory and replace with minimal version \
        cd /catkin_ws && \
        rm -rf src && \
        mv /tmp/catkin_ws_minimal/src . && \
        # Clean up \
        rm /tmp/keep_files.txt && \
        # Display what's left (for debugging) \
        echo "=== Remaining source files ===" && \
        du -sh src && \
        find src -type f | wc -l && \
        echo "files remaining"; \
    else \
        echo "=== Building DEVELOPMENT image - keeping all source code ==="; \
    fi

# Set up environment in bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# Copy custom bashrc for additional configuration
COPY docker/.bashrc /root/.bashrc_custom
RUN cat /root/.bashrc_custom >> /root/.bashrc

# Set working directory
WORKDIR /catkin_ws

# Expose ROS and MAVROS ports
EXPOSE 11311 14540 14557

# Default environment variables (can be overridden at runtime)
ENV PX4_SIM_HOST=localhost
ENV PX4_SIM_MODEL=iris

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
