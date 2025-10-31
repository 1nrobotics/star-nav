# Star Nav - Docker Setup

This repository contains a unified Docker setup for building and deploying all ROS packages in the Star Nav system, including:
- **px4ctrl**: PX4 flight controller interface
- **planner**: EGO planner for autonomous obstacle avoidance
- **vk180-ros**: VK180 VIO sensor integration and eCAL bridge
- **shiviz_px4_nav**: Waypoint navigation system
- **utils**: Common utilities (quadrotor_msgs, uav_utils, DecompROS, etc.)
- **realsense-ros**: Intel RealSense camera support

## Architecture Support

The Docker setup supports multiple architectures:
- **x86_64/AMD64**: Standard desktop/laptop development
- **ARM64**: NVIDIA Jetson NX/Orin, Raspberry Pi 4, Radxa boards

## Quick Start

### x86_64 Local Development

```bash
# Build the Docker image
cd /path/to/star-nav
docker build -t 1nrobotics/star-nav:local .

# Run interactive development container
./docker/interactive-deploy.sh

# Or deploy with VK180 autonomous flight system
./docker/local-deploy.sh
```

### NVIDIA Jetson NX Deployment

```bash
# On Jetson NX, build natively (automatic ARM64)
cd /path/to/star-nav
docker build -t 1nrobotics/star-nav:jetson .

# Deploy with GPU acceleration
./docker/jetson-deploy.sh
```

**Important Notes:**
- The standard `Dockerfile` works on Jetson but may have eCAL ARM64 compatibility issues
- For VK180 systems: Verify eCAL ARM64 packages are available
- For non-VK180 systems: Use `Dockerfile.jetson-no-ecal` (skips eCAL installation)
- For best performance: Use `Dockerfile.jetson` (NVIDIA L4T optimized base)

## Detailed Deployment Guides

### Jetson NX/Orin Setup

#### Prerequisites
1. **NVIDIA Container Runtime** (for GPU acceleration):
   ```bash
   # Install nvidia-docker2
   sudo apt-get update
   sudo apt-get install -y nvidia-docker2
   
   # Restart Docker daemon
   sudo systemctl restart docker
   
   # Verify GPU access
   docker run --rm --runtime nvidia nvcr.io/nvidia/l4t-base:r35.4.1 nvidia-smi
   ```

2. **X11 Display Access** (for RViz):
   ```bash
   # Allow Docker containers to access display
   xhost +local:docker
   ```

#### Building on Jetson

The Dockerfile supports multi-architecture builds. When building on Jetson NX, it automatically uses ARM64:

```bash
cd /path/to/star-nav

# Standard build (uses default base image)
docker build -t 1nrobotics/star-nav:jetson .

# Or use Jetson-optimized Dockerfile (includes NVIDIA L4T base)
docker build -f Dockerfile.jetson -t 1nrobotics/star-nav:jetson .
```

**Build time**: Approximately 15-25 minutes depending on Jetson model and SD card speed.

#### Deployment Options

**Option 1: Using deployment script (Recommended)**
```bash
cd /path/to/star-nav
bash docker/jetson-deploy.sh
```

**Option 2: Manual Docker run**
```bash
docker run -d \
    --name star_nav \
    --runtime nvidia \
    --network host \
    --privileged \
    -v /dev:/dev:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e DISPLAY=:0 \
    -e ROS_MASTER_URI=http://localhost:11311 \
    1nrobotics/star-nav:jetson \
    bash -c "source /catkin_ws/devel/setup.bash && roslaunch vk180_ros vk180_autonomous_flight.launch use_sim:=false"
```

#### Jetson-Specific Features

1. **GPU Acceleration**: The `--runtime nvidia` flag enables:
   - CUDA-accelerated point cloud processing
   - TensorRT optimizations
   - GPU-accelerated computer vision

2. **Hardware Access**: Full `/dev` mount provides:
   - Serial port access (`/dev/ttyUSB*`, `/dev/ttymxc*`) for PX4
   - USB camera access for VK180 or RealSense
   - GPIO and peripheral access

3. **Performance Monitoring**:
   ```bash
   # Jetson stats (CPU, GPU, RAM, Temp)
   sudo tegrastats
   
   # GPU utilization
   nvidia-smi
   
   # Container resource usage
   docker stats star_nav
   ```

#### Troubleshooting Jetson Deployment

**Problem: "unknown runtime 'nvidia'"**
```bash
# Solution: Install nvidia-docker2
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

**Problem: RViz doesn't display**
```bash
# Solution: Grant X11 access
xhost +local:docker
export DISPLAY=:0
```

**Problem: PX4 serial port not accessible**
```bash
# Solution: Add user to dialout group
sudo usermod -aG dialout $USER
# Logout and login again
```

**Problem: VK180 eCAL connection fails**
```bash
# Solution: Check network configuration
# eCAL uses UDP multicast - ensure firewall allows it
sudo ufw allow from 224.0.0.0/4
```

#### Dockerfile Selection for Jetson

The repository provides multiple Dockerfiles optimized for different use cases:

| Dockerfile | Base Image | eCAL Support | GPU Ready | Best For |
|------------|------------|--------------|-----------|----------|
| **Dockerfile** | ros:noetic-ros-core-focal | ✅ Yes (may need ARM64 check) | ⚠️ Manual | General use, x86_64 & ARM64 |
| **Dockerfile.jetson** | nvcr.io/nvidia/l4t-base | ✅ Yes | ✅ Yes | VK180 on Jetson with GPU |
| **Dockerfile.jetson-no-ecal** | ros:noetic-ros-core-focal | ❌ No | ⚠️ Manual | Non-VK180 systems on Jetson |

**Choosing the right Dockerfile:**

1. **For VK180 on Jetson** (Recommended):
   ```bash
   docker build -f Dockerfile.jetson -t 1nrobotics/star-nav:jetson .
   ```
   - NVIDIA L4T base image (optimized for Jetson)
   - GPU acceleration enabled
   - eCAL with ARM64 support
   - Best performance for autonomous flight with VK180

2. **For non-VK180 systems on Jetson**:
   ```bash
   docker build -f Dockerfile.jetson-no-ecal -t 1nrobotics/star-nav:jetson .
   ```
   - Skips eCAL installation (faster build)
   - Smaller image size
   - Use when VK180 sensor is not needed
   - Good for EGO planner + RealSense setups

3. **For x86_64 development**:
   ```bash
   docker build -t 1nrobotics/star-nav:local .
   ```
   - Standard Dockerfile works perfectly
   - All packages including VK180 support

4. **Testing on Jetson before committing**:
   ```bash
   # Try standard Dockerfile first
   docker build -t 1nrobotics/star-nav:test .
   
   # If eCAL fails, use jetson-specific version
   docker build -f Dockerfile.jetson -t 1nrobotics/star-nav:jetson .
   ```

**eCAL ARM64 Compatibility Notes:**
- The Vilota installer in the standard Dockerfile may not provide ARM64 binaries
- If build fails at eCAL installation step, use `Dockerfile.jetson` which builds eCAL from source
- Alternatively, skip VK180 package: `touch vk180-ros/CATKIN_IGNORE` before building

### Build Options

```bash
# Build with default settings (latest tag, AMD64, Release)
./docker/build.sh

# Build with specific tag
./docker/build.sh v1.0.0

# Build for ARM64 (Radxa/Raspberry Pi)
./docker/build.sh latest linux/arm64

# Debug build
./docker/build.sh debug linux/amd64 Debug

# Clean old images before building
./docker/build.sh latest linux/amd64 Release --clean

# Skip tests
./docker/build.sh latest linux/amd64 Release --no-test

# Push to registry after build
./docker/build.sh v1.0.0 linux/amd64 Release --push
```

## Deployment

### Local Deployment

Deploy all services locally:
```bash
./docker/deploy.sh --local

# Or simply
docker-compose up -d
```

### Remote Deployment

Deploy to a remote device (Radxa, Raspberry Pi, etc.):

```bash
# First, setup Docker on remote host
./docker/deploy.sh --setup-remote user@remote-host

# Deploy to remote
./docker/deploy.sh latest user@remote-host 1

# Or use the flag
./docker/deploy.sh --remote latest user@192.168.1.100 1
```

### Deployment Modes

#### Simulation Mode
Run with PX4 SITL simulation:
```bash
./docker/deploy.sh --simulation
# Or
docker-compose --profile simulation up -d
```

#### Development Mode
Interactive development environment:
```bash
./docker/deploy.sh --development
# Or
docker-compose --profile development up -d
# Access the container
docker-compose exec star_nav_dev bash
```

#### PX4 Controller Only
Run only the PX4 controller:
```bash
./docker/deploy.sh --px4ctrl
# Or
docker-compose --profile px4ctrl up -d
```

#### Visualization
Run with RViz visualization:
```bash
docker-compose --profile visualization up -d
```

## Docker Compose Services

### Main Services

- **star_nav**: Main navigation service (runs shiviz_px4_nav by default)
- **px4ctrl**: PX4 controller service (profile: px4ctrl)
- **star_nav_dev**: Development environment (profile: development)

### Optional Services

- **px4_sitl**: PX4 SITL simulation (profile: simulation)
- **mavproxy**: Ground Control Station proxy (profile: gcs)
- **rviz**: RViz visualization (profile: visualization)

### Service Management

```bash
# Start main service
docker-compose up -d

# Start with specific profiles
docker-compose --profile simulation up -d
docker-compose --profile development up -d
docker-compose --profile visualization up -d

# View logs
docker-compose logs -f

# View specific service logs
docker-compose logs -f star_nav

# Stop all services
docker-compose down

# Restart a service
docker-compose restart star_nav

# Execute commands in container
docker-compose exec star_nav bash
```

## Environment Variables

Set these in your shell or in a `.env` file:

```bash
# Docker image configuration
export IMAGE_TAG=latest              # Docker image tag
export REGISTRY=ghcr.io              # Docker registry

# ROS configuration
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export ROS_IP=localhost

# PX4 configuration
export PX4_SIM_HOST=localhost        # PX4 SITL host
export PX4_SIM_MODEL=iris            # PX4 aircraft model
export VEHICLE_ID=1                  # Vehicle/drone ID

# Display (for GUI applications)
export DISPLAY=:0
```

## Volume Mounts

The Docker setup mounts the following directories for development:

```yaml
# Source code (read-write)
./px4ctrl -> /catkin_ws/src/px4ctrl
./shiviz_px4_nav -> /catkin_ws/src/shiviz_px4_nav
./utils -> /catkin_ws/src/utils

# Logs
./logs -> /catkin_ws/logs

# Device access
/dev -> /dev  # For serial communication with flight controller
```

## Container Aliases

Inside the container, several convenient aliases are available:

### ROS Aliases
```bash
rosenv                  # Show ROS environment variables
catkin_build_release    # Build workspace in Release mode
catkin_clean_all        # Clean all build artifacts
```

### Navigation Aliases
```bash
nav_launch             # Launch navigation system
nav_takeoff            # Command takeoff
nav_land               # Command landing
nav_mission            # Start waypoint mission
nav_stop               # Stop mission
nav_status             # Monitor navigation status
```

### PX4 Controller Aliases
```bash
ctrl_launch            # Launch PX4 controller
thrust_cal             # Launch thrust calibration
px4_arm                # Arm the drone
px4_disarm             # Disarm the drone
px4_mode_offboard      # Set OFFBOARD mode
px4_mode_manual        # Set MANUAL mode
```

### Monitoring Aliases
```bash
px4_state              # Echo PX4 state
px4_pose               # Echo local position
px4_velocity           # Echo velocity
```

### Development Aliases
```bash
kill_ros               # Kill all ROS processes
restart_navigation     # Restart navigation node
restart_px4ctrl        # Restart PX4 controller
```

## Building on ARM64 Devices

For deployment on Radxa or Raspberry Pi:

```bash
# Build ARM64 image locally (requires buildx)
docker buildx build --platform linux/arm64 -t 1nrobotics/star-nav:arm64 .

# Or build directly on the device
ssh user@device
cd /path/to/star-nav
docker build -t 1nrobotics/star-nav:arm64 .
```

## Dockerfile Architecture

### Multi-Architecture Support

The Dockerfile uses a build argument to support multiple platforms:

```dockerfile
# Multi-architecture support: works for x86_64 and ARM64 (Jetson)
ARG BASE_IMAGE=ros:noetic-ros-core-focal
FROM ${BASE_IMAGE}
```

**Default behavior:**
- On x86_64: Uses `ros:noetic-ros-core-focal` (AMD64)
- On ARM64: Uses `ros:noetic-ros-core-focal` (ARM64)

**For Jetson-optimized build:**
Use `Dockerfile.jetson` which starts from NVIDIA L4T base:
```bash
docker build -f Dockerfile.jetson -t 1nrobotics/star-nav:jetson .
```

### Build Process Overview

The Dockerfile follows these stages:

1. **Base Setup** (Lines 1-31)
   - ROS Noetic base image (Ubuntu 20.04)
   - Essential build tools (vim, git, wget, etc.)
   - rosdep initialization

2. **ROS Dependencies** (Lines 33-69)
   - ROS base packages
   - MAVROS and MAVLink
   - Navigation packages (PCL, Eigen, cv_bridge, etc.)
   - Additional tools (roslint, xacro, ddynamic_reconfigure)

3. **System Libraries** (Lines 71-78)
   - Eigen3 for linear algebra
   - PCL for point cloud processing
   - YAML-CPP for configuration files

4. **eCAL Installation** (Lines 80-107)
   - Vilota Software Installer for eCAL
   - eCAL Python bindings
   - CapnProto for serialization
   - Required for VK180 communication

5. **Hardware Support** (Lines 109-115)
   - Movidius udev rules
   - MAVROS geographic datasets

6. **Workspace Build** (Lines 117-133)
   - Copy source code
   - Run `catkin_make` with Release optimization
   - Install package dependencies via rosdep

7. **Environment Setup** (Lines 135-148)
   - Configure ROS environment variables
   - Setup bashrc with custom aliases
   - Expose ROS/MAVROS ports

### Key Features

**VK180 Support:**
- eCAL for inter-process communication
- CapnProto for efficient serialization
- Point cloud processing with PCL

**PX4 Integration:**
- MAVROS for MAVLink communication
- Serial port support via privileged mode
- Geographic datasets for GPS

**GPU Acceleration (Jetson only):**
- Uses `--runtime nvidia` flag
- Enables CUDA and TensorRT
- Optimized for point cloud processing

**Skipped Packages:**
- `realsense2_camera` (requires Intel RealSense SDK)
  - Has `CATKIN_IGNORE` file
  - Not needed for VK180-based system

### Customization

**To add packages:**
```dockerfile
# Add to ROS dependencies section
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-your-package \
    && rm -rf /var/lib/apt/lists/*
```

**To change build type:**
```dockerfile
# In workspace build section, change:
RUN catkin_make -DCMAKE_BUILD_TYPE=Release
# To:
RUN catkin_make -DCMAKE_BUILD_TYPE=Debug
```

**To use different base image:**
```bash
# Build with custom base
docker build --build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-base:r35.4.1 -t star-nav:custom .
```

### Jetson Compatibility

**Will the standard Dockerfile work on Jetson?**

✅ **YES**, with caveats:

**What works on Jetson:**
- All ROS Noetic packages (ARM64 versions available)
- MAVROS and MAVLink
- Navigation stack (PCL, Eigen, cv_bridge, etc.)
- EGO planner compilation
- px4ctrl and trajectory server
- Standard ROS functionality

**Potential issues on Jetson:**

1. **eCAL Installation** (Lines 85-97):
   ```dockerfile
   RUN python3 /tmp/installer.py -s -u deployment -p vilota-dep --no-update-script
   ```
   - ⚠️ Vilota installer may not have ARM64 binaries for eCAL
   - **Solution 1**: Use `Dockerfile.jetson` (builds eCAL from source)
   - **Solution 2**: Use `Dockerfile.jetson-no-ecal` (skips VK180)
   - **Solution 3**: Manually verify/build eCAL for ARM64

2. **GPU Acceleration** (Not enabled by default):
   - Standard Dockerfile doesn't include NVIDIA runtime optimizations
   - **Solution**: Use `Dockerfile.jetson` with L4T base for GPU support
   - Deploy with `--runtime nvidia` flag (handled by `jetson-deploy.sh`)

3. **VK180 Package**:
   - Requires eCAL for sensor data bridge
   - If eCAL fails, VK180 package won't build
   - **Solution**: Add `CATKIN_IGNORE` to vk180-ros if not needed

**Recommended approach:**

For VK180 systems on Jetson:
```bash
# Use Jetson-optimized Dockerfile
docker build -f Dockerfile.jetson -t 1nrobotics/star-nav:jetson .
```

For non-VK180 systems on Jetson:
```bash
# Use lightweight version without eCAL
docker build -f Dockerfile.jetson-no-ecal -t 1nrobotics/star-nav:jetson .

# Or skip VK180 package in standard build
touch vk180-ros/CATKIN_IGNORE
docker build -t 1nrobotics/star-nav:jetson .
```

Test standard Dockerfile first:
```bash
# Try building - will show if eCAL is compatible
docker build -t 1nrobotics/star-nav:test .

# If successful, proceed with deployment
# If fails at eCAL step, use Dockerfile.jetson instead
```

### Build Performance

**x86_64 Desktop:**
- Build time: 10-15 minutes
- Image size: ~4.5 GB

**NVIDIA Jetson NX:**
- Build time: 15-25 minutes (native ARM64)
- Build time: 30-40 minutes (with Dockerfile.jetson - includes eCAL from source)
- Image size: ~4.8 GB
- Recommend: Fast SD card or NVMe SSD

**Optimization tips:**
- Use BuildKit: `DOCKER_BUILDKIT=1 docker build ...`
- Layer caching: Rebuild only changed layers
- Multi-stage builds: Separate build and runtime images (future improvement)

## Deployment Script Comparison

| Feature | local-deploy.sh | jetson-deploy.sh | interactive-deploy.sh |
|---------|----------------|------------------|----------------------|
| **Platform** | x86_64 desktop | NVIDIA Jetson | Any (development) |
| **Mode** | Background daemon | Background daemon | Interactive shell |
| **GPU Support** | No | Yes (`--runtime nvidia`) | Optional |
| **Launch File** | vk180_autonomous_flight.launch | vk180_autonomous_flight.launch | None (manual) |
| **Auto-build** | Yes | Yes | Yes |
| **Simulation** | Configurable | use_sim:=false | Manual |
| **Volume Mounts** | All packages | All packages | All packages |

## Launch File Comparison
cd ~/star_nav
docker build -t 1nrobotics/star-nav:local .
```

## Troubleshooting

### X11 Display Issues

If GUI applications don't work:
```bash
# On host
xhost +local:docker

# Set DISPLAY correctly
export DISPLAY=:0
```

### Permission Issues with /dev

Ensure the container runs with `--privileged` flag or add user to dialout group:
```bash
sudo usermod -aG dialout $USER
```

### Network Issues

If ROS nodes can't communicate:
```bash
# Check ROS environment
docker-compose exec star_nav bash -c "rosenv"

# Verify network mode is host
docker-compose config | grep network_mode
```

### Build Cache Issues

Clear Docker build cache:
```bash
docker builder prune
docker system prune -a
```

## Development Workflow

1. **Make code changes** in your local workspace (`px4ctrl/`, `shiviz_px4_nav/`, `utils/`)

2. **Rebuild inside container**:
   ```bash
   docker-compose exec star_nav bash
   cd /catkin_ws
   catkin build
   source devel/setup.bash
   ```

3. **Or rebuild image** for production:
   ```bash
   ./docker/build.sh
   ```

4. **Test changes**:
   ```bash
   docker-compose restart star_nav
   docker-compose logs -f star_nav
   ```

## Continuous Integration

The Docker setup can be integrated with CI/CD pipelines:

```yaml
# Example GitHub Actions workflow
- name: Build Docker Image
  run: ./docker/build.sh ${{ github.sha }} linux/amd64 Release --push

- name: Test Image
  run: docker run --rm 1nrobotics/star-nav:${{ github.sha }} rospack list
```

## File Structure

```
catkin_ws/src/
├── Dockerfile                 # Multi-package Docker image
├── docker-compose.yml         # Service orchestration
├── .dockerignore             # Files to exclude from build
├── docker/                   # Docker utilities
│   ├── build.sh             # Build script
│   ├── deploy.sh            # Deployment script
│   ├── local-deploy.sh      # Quick local deploy
│   ├── interactive-deploy.sh # Interactive container
│   └── .bashrc              # Container shell config
├── px4ctrl/                 # PX4 controller package
├── shiviz_px4_nav/          # Navigation package
└── utils/                   # Utility packages
```

## Support

For issues or questions:
- Check container logs: `docker-compose logs -f`
- Inspect container: `docker-compose exec star_nav bash`
- Review build output: `./docker/build.sh 2>&1 | tee build.log`

## Migration from Old Setup

If you're migrating from the old `shiviz_px4_nav/Dockerfile`:

1. The new Dockerfile builds **all packages** (px4ctrl, shiviz_px4_nav, utils)
2. Docker files are now at the **root level** (`catkin_ws/src/`)
3. Use `./docker/build.sh` instead of building from `shiviz_px4_nav/`
4. All docker-compose profiles are still available
5. The old setup can be removed or kept as backup

## License

See individual package LICENSE files for details.
