FROM ros:noetic-ros-core-focal
LABEL maintainer="1nrobotics@github"
LABEL description="Docker image for Star Nav - Multi-package ROS Navigation System (px4ctrl, shiviz_px4_nav, utils)"

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

# Install eCAL using Vilota software installer
# Copy the installer from shiviz_px4_nav package
COPY shiviz_px4_nav/tools/VilotaSoftwareInstaller_v2.0-bba1a93f.py /tmp/installer.py
# Ensure helper tools needed by the Vilota installer are available during build
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-requests \
        software-properties-common \
        sudo \
        iputils-ping \
        apt-transport-https \
        ca-certificates \
        gnupg && \
    rm -rf /var/lib/apt/lists/*

# Install eCAL, capnp and dependencies with Vilota installer (requires credentials to be passed as build args)
RUN apt-get update && \
    python3 /tmp/installer.py -s -u deployment -p vilota-dep --no-update-script && \
    rm -rf /var/lib/apt/lists/*

# Install eCAL Python bindings explicitly
RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-ecal5 && \
    rm -rf /var/lib/apt/lists/* || \
    echo "Warning: python3-ecal5 package not available"

# Verify eCAL and pycapnp installation
RUN python3 -c "import ecal; print('eCAL imported successfully')" || echo "Warning: eCAL import failed"
RUN python3 -c "import capnp; print(f'pycapnp {capnp.__version__} imported successfully')" || echo "Warning: pycapnp import failed"

# Ensure udev rules directory exists and add Movidius udev rule
RUN mkdir -p /etc/udev/rules.d && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' > /etc/udev/rules.d/80-movidius.rules || true

# Install MAVROS geographic datasets
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && bash ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"]

# Create workspace and copy all source code
WORKDIR /catkin_ws/src/
COPY . /catkin_ws/src/

# Copy eCAL configuration
COPY shiviz_px4_nav/tools/ecal.ini /etc/ecal/ecal.ini

# Initialize and build the workspace
WORKDIR /catkin_ws/
RUN source /opt/ros/noetic/setup.bash \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && catkin_make -DCMAKE_BUILD_TYPE=Release

# Set up environment in bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc && \
    echo "export ROS_HOSTNAME=localhost" >> /root/.bashrc && \
    echo "export ROS_IP=localhost" >> /root/.bashrc

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
ENV VEHICLE_ID=1

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
