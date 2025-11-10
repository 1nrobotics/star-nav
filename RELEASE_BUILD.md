# Release Build Configuration

This branch (`feature/build-for-release`) creates a production-ready Docker image with minimal size by removing source code after compilation.

## What's Different from Development Build

### Dockerfile Changes
- **After `catkin_make`**, the source code is cleaned up
- **Keeps only runtime-necessary files**:
  - Launch files (`*.launch`, `*.xml` for urdf/xacro)
  - Configuration files (`*.yaml`, `*.rviz`, `*.ini`)
  - Python scripts (in `scripts/` directories)
  - Shell scripts (`*.sh`)
  - Message/Service/Action definitions
  - Package manifests (`package.xml` - required for roslaunch)
- **Removes**:
  - C++ source files (`*.cpp`, `*.h`, `*.hpp`)
  - CMakeLists.txt files
  - Build artifacts and temporary files
  - Unnecessary documentation

### Deployment Scripts

#### Development Deployment: `docker/local-deploy.sh`
- Mounts source code directories from host into container
- Allows live code editing and quick rebuilds
- Uses `IMAGE_TAG=local`
- **Use for**: Development, debugging, testing new features

#### Release Deployment: `docker/release-deploy.sh` ✨ NEW
- No source code volume mounts
- Runs entirely from compiled binaries in the image
- Uses `IMAGE_TAG=release`
- Automatically launches the navigation stack
- **Use for**: Production deployments, field operations

## Usage

### Building the Release Image
```bash
cd /home/yc/catkin_ws/src
docker build -t 1nrobotics/star-nav:release .
```

### Deploying Release Build
```bash
./docker/release-deploy.sh
```

Or with custom serial device:
```bash
DEVICE=/dev/ttyACM0 ./docker/release-deploy.sh
```

### Deploying Development Build
```bash
./docker/local-deploy.sh
```

## Benefits

1. **Smaller Image Size**: Removes ~hundreds of MB of source code
2. **IP Protection**: Source code is not accessible in the deployed container
3. **Cleaner Runtime**: No unnecessary files in the container
4. **Production Ready**: Optimized for deployment, not development

## Important Notes

- The release image is **immutable** - you cannot edit source code at runtime
- To make code changes, you must:
  1. Edit source on the host
  2. Rebuild the Docker image
  3. Redeploy the container
- Use the development deployment (`local-deploy.sh`) for active development
- Use the release deployment (`release-deploy.sh`) for testing deployment scenarios or production

## Verification

After building, you can verify source code removal:
```bash
docker run --rm 1nrobotics/star-nav:release bash -c "find /catkin_ws/src -name '*.cpp' | wc -l"
# Should output: 0

docker run --rm 1nrobotics/star-nav:release bash -c "du -sh /catkin_ws/src"
# Should show significantly reduced size
```

## Troubleshooting

If the release build fails to launch:
1. Check that all necessary runtime files are preserved in Dockerfile
2. Verify launch files and configs are still present
3. Check logs: `docker logs star_nav`
4. Test with development build first to isolate the issue
