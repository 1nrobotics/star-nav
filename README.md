# Star Nav - Multi-package ROS Navigation System

A unified ROS navigation system for autonomous drone flight control, combining PX4 controller interface, waypoint navigation, and common utilities.

## 🚀 Quick Start

### Clone Repository

```bash
git clone https://github.com/1nrobotics/star-nav.git
cd star-nav
```

The `main` branch is the default and stable branch for this repository.

### Docker Deployment (Recommended)

For quick setup with Docker:

```bash
# Build Docker image
./docker/build.sh

# Deploy locally
./docker/local-deploy.sh

# Or use interactive development mode
./docker/interactive-deploy.sh
```

See [QUICKSTART.md](./QUICKSTART.md) for more options.

## 📦 What's Included

This repository contains all ROS packages for the Star Nav system:

- **px4ctrl** - PX4 flight controller interface
- **shiviz_px4_nav** - Waypoint navigation system  
- **utils** - Common utilities (quadrotor_msgs, uav_utils, DecompROS, etc.)
- **realsense-ros** - RealSense camera integration
- **planner** - Path planning components

## 📚 Documentation

- [Quick Start Guide](./QUICKSTART.md) - Get started quickly
- [Docker Setup](./DOCKER_README.md) - Comprehensive Docker documentation
- [Migration Summary](./MIGRATION_SUMMARY.md) - Changes from previous setup
- [Test Summary](./TEST_SUMMARY.md) - Testing documentation

## 🏗️ Building

### Using Docker (Recommended)

```bash
./docker/build.sh
```

### Manual Build

Requires ROS Noetic on Ubuntu 20.04:

```bash
# Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build workspace
catkin build

# Source workspace
source devel/setup.bash
```

## 🚁 Deployment

### Local Development

```bash
# Interactive container for development
./docker/interactive-deploy.sh

# Background service
./docker/local-deploy.sh
```

### Remote Deployment (Radxa, Raspberry Pi, etc.)

```bash
# Setup remote host
./docker/deploy.sh --setup-remote user@remote-host

# Deploy to remote device
./docker/deploy.sh latest user@remote-host 1
```

### Deployment Modes

- **Simulation**: `./docker/deploy.sh --simulation`
- **Development**: `./docker/deploy.sh --development`
- **PX4 Controller Only**: `./docker/deploy.sh --px4ctrl`

## 🌿 Branching Strategy

- **main** - Default and stable branch for production use
- Feature branches follow the pattern: `feature/description` or `copilot/description`

## 🤝 Contributing

1. Fork the repository
2. Create your feature branch from `main`: `git checkout -b feature/my-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin feature/my-feature`
5. Submit a pull request to the `main` branch

## 📄 License

See individual package LICENSE files for details.

## 🆘 Support

For issues or questions:
- Check [DOCKER_README.md](./DOCKER_README.md) for Docker-specific help
- View container logs: `docker-compose logs -f`
- Inspect running containers: `docker-compose ps`

## 🔗 Related Repositories

- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
- [MAVROS](https://github.com/mavlink/mavros)
- [RealSense ROS](https://github.com/IntelRealSense/realsense-ros)
