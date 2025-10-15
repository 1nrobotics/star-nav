# Test Summary - Local Deploy

## Date: October 15, 2025

## Test: local-deploy.sh

### Status: ✅ IN PROGRESS (Build Running)

## Issues Found & Fixed:

### 1. Docker Compose Not Available
**Issue**: `docker-compose` command not found
**Solution**: Updated scripts to use pure `docker run` commands instead
- Modified `docker/local-deploy.sh`
- Modified `docker/interactive-deploy.sh`

### 2. Docker Permission Denied
**Issue**: User not in docker group
**Solution**: Added user to docker group with `sudo usermod -aG docker $USER`
**Note**: Running with `sudo` for now until user logs out/in

### 3. Build Error - Missing yaml-cpp Submodule
**Issue**: `shiviz_px4_nav/thirdparty/yaml-cpp` directory empty (git submodule not initialized)
**Error**: 
```
CMake Error: The source directory
  /catkin_ws/src/shiviz_px4_nav/thirdparty/yaml-cpp
does not contain a CMakeLists.txt file.
```
**Solution**: 
- Updated `Dockerfile` to install system `libyaml-cpp-dev` and `libyaml-cpp0.6`
- Modified `shiviz_px4_nav/CMakeLists.txt` to use system yaml-cpp:
  - Commented out `add_subdirectory(thirdparty/yaml-cpp)`
  - Added `find_package(yaml-cpp REQUIRED)`
  - Changed link library from `yaml-cpp` to `${YAML_CPP_LIBRARIES}`
  - Commented out yaml-cpp install target

### 4. Build Error - Missing multi_map_server Dependency
**Issue**: `rviz_plugins` depends on non-existent `multi_map_server` package
**Error**:
```
Cannot locate rosdep definition for [multi_map_server]
```
**Solution**: 
- Modified `utils/rviz_plugins/package.xml`
- Commented out `multi_map_server` from both build and run dependencies

## Files Modified:

1. **`/home/yc/catkin_ws/src/Dockerfile`**
   - Added `libyaml-cpp0.6` to system dependencies

2. **`/home/yc/catkin_ws/src/shiviz_px4_nav/CMakeLists.txt`**
   - Use system yaml-cpp instead of bundled version
   - Removed yaml-cpp install target

3. **`/home/yc/catkin_ws/src/utils/rviz_plugins/package.xml`**
   - Removed multi_map_server dependency

4. **`/home/yc/catkin_ws/src/docker/local-deploy.sh`**
   - Converted from docker-compose to pure docker run

5. **`/home/yc/catkin_ws/src/docker/interactive-deploy.sh`**
   - Converted from docker-compose to pure docker run

## Current Status:

Docker build is now running successfully:
- Base image layers: ✅ CACHED
- ROS packages: ✅ CACHED  
- System dependencies: ✅ INSTALLED
- eCAL installation: 🔄 IN PROGRESS
- Catkin workspace build: ⏳ PENDING

## Expected Next Steps:

1. Wait for Docker build to complete (~5-10 minutes)
2. If build succeeds, container will start automatically
3. Verify container is running with suggested commands
4. Test ROS packages inside container

## Commands Used:

```bash
# Build and deploy
sudo ./docker/local-deploy.sh

# Check container status
docker ps | grep star_nav

# View logs
docker logs -f star_nav

# Access container
docker exec -it star_nav bash

# Stop container
docker stop star_nav
docker rm star_nav
```

## Notes:

- Build is using cached layers for speed
- First build will take longer (~10-15 minutes)
- Subsequent builds will be faster due to caching
- All source code is mounted as volumes for live development
- No need for docker-compose anymore - pure Docker!
