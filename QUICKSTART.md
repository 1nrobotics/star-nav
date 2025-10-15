# Star Nav - Quick Start Guide

## 🚀 Quick Start

### Option 1: Interactive Development (Recommended for first time)
```bash
cd /home/yc/catkin_ws/src
./docker/interactive-deploy.sh
```

### Option 2: Background Service
```bash
cd /home/yc/catkin_ws/src
./docker/local-deploy.sh
```

### Option 3: Build from Scratch
```bash
cd /home/yc/catkin_ws/src
./docker/build.sh
docker-compose up -d
```

## 📦 What's Included

This Docker setup builds all ROS packages:
- ✈️ **px4ctrl** - PX4 flight controller
- 🧭 **shiviz_px4_nav** - Waypoint navigation
- 🔧 **utils** - Common utilities (quadrotor_msgs, uav_utils, DecompROS, etc.)

## 🎯 Common Commands

```bash
# Build Docker image
./docker/build.sh

# Deploy locally
./docker/deploy.sh --local

# Development mode
./docker/deploy.sh --development

# With PX4 simulation
./docker/deploy.sh --simulation

# View logs
docker-compose logs -f

# Enter container
docker-compose exec star_nav bash

# Stop all services
docker-compose down
```

## 📚 Full Documentation

See [DOCKER_README.md](./DOCKER_README.md) for complete documentation.

## 🗂️ File Structure

```
/home/yc/catkin_ws/src/
├── Dockerfile              # Builds all packages
├── docker-compose.yml      # Service orchestration
├── .dockerignore          # Build exclusions
├── docker/                # Scripts
│   ├── build.sh          # Build image
│   ├── deploy.sh         # Deploy services
│   ├── local-deploy.sh   # Quick local start
│   └── interactive-deploy.sh  # Dev container
├── px4ctrl/              # PX4 controller
├── shiviz_px4_nav/       # Navigation
└── utils/                # Utilities
```

## 🔧 Inside the Container

Useful aliases available:
- `nav_launch` - Start navigation
- `ctrl_launch` - Start PX4 controller  
- `px4_arm` - Arm drone
- `nav_status` - Check status

## 💡 Tips

1. All source code is mounted - changes reflect immediately
2. Use `catkin build` inside container to rebuild
3. Logs are saved to `./logs/`
4. Press `Ctrl+D` or type `exit` to leave interactive mode

## ❓ Need Help?

```bash
# Check build script options
./docker/build.sh --help

# Check deployment options
./docker/deploy.sh --help

# View container logs
docker-compose logs -f star_nav
```
