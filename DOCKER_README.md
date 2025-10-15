# Star Nav - Docker Setup

This repository contains a unified Docker setup for building and deploying all ROS packages in the Star Nav system, including:
- **px4ctrl**: PX4 flight controller interface
- **shiviz_px4_nav**: Waypoint navigation system
- **utils**: Common utilities (quadrotor_msgs, uav_utils, DecompROS, etc.)

## Quick Start

### Local Development

```bash
# Build the Docker image
cd /path/to/catkin_ws/src
./docker/build.sh

# Run interactive development container
./docker/interactive-deploy.sh

# Or use docker-compose for background services
./docker/local-deploy.sh
```

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
