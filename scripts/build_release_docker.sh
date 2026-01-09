#!/bin/bash

# Build a release Docker image from a packed workspace
# This script:
# 1. Builds the workspace in a builder container
# 2. Packs it (removes proprietary source code)
# 3. Creates a runtime-only Docker image with the packed workspace

set -e

SCRIPT_DIR=$(dirname "$(realpath "$0")")
REPO_ROOT=$(realpath "$SCRIPT_DIR/..")
BUILD_CONTEXT_DIR="$REPO_ROOT/build_context_release"
IMAGE_NAME="${IMAGE_NAME:-1nrobotics/star-nav}"
IMAGE_TAG="${IMAGE_TAG:-runtime}"

echo "=== Star Nav Release Build ==="
echo "Building workspace and creating runtime Docker image..."
echo ""

# Clean up any previous build context
rm -rf "$BUILD_CONTEXT_DIR"
mkdir -p "$BUILD_CONTEXT_DIR"

echo "Step 1: Building workspace in Docker builder container..."
# Build using the existing Dockerfile.install
cd "$REPO_ROOT"
docker build -t star-nav-builder:temp -f Dockerfile.install .

echo ""
echo "Step 2: Extracting built workspace from builder container..."
# Create temporary container and copy workspace out
docker create --name star-nav-builder-temp star-nav-builder:temp

# Try to copy install directory first, fall back to devel if it doesn't exist
if docker cp star-nav-builder-temp:/catkin_ws/install "$BUILD_CONTEXT_DIR/" 2>/dev/null; then
    echo "  Copied install/ directory"
else
    echo "  install/ not found, copying devel/ directory instead"
    docker cp star-nav-builder-temp:/catkin_ws/devel "$BUILD_CONTEXT_DIR/install"
fi

docker cp star-nav-builder-temp:/catkin_ws/src "$BUILD_CONTEXT_DIR/"
docker rm star-nav-builder-temp

echo ""
echo "Step 3: Cleaning proprietary source code from workspace..."
# Define packages to clean (remove source code but keep launch/config)
PACKAGES_TO_CLEAN=("vk180-ros" "planner/exploration_manager" "rviz_gcs_plugin")

cd "$BUILD_CONTEXT_DIR/src"
for pkg in "${PACKAGES_TO_CLEAN[@]}"; do
    if [ -d "$pkg" ]; then
        echo "  Cleaning $pkg..."
        # Remove C++ source and build files
        find "$pkg" -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" -o -name "CMakeLists.txt" \) -delete
        # Remove src and include directories
        rm -rf "$pkg/src" "$pkg/include" 2>/dev/null || true
    fi
done

echo ""
echo "Step 4: Preparing build context with necessary files..."
# Copy necessary files for Docker build (exclude build_context_release itself to avoid recursion)
cp -r "$REPO_ROOT/docker" "$BUILD_CONTEXT_DIR/"
cp "$REPO_ROOT/Dockerfile.runtime" "$BUILD_CONTEXT_DIR/"

# Modify .bashrc to use install/setup.bash instead of devel/setup.bash
sed -i 's|/catkin_ws/devel/setup.bash|/catkin_ws/install/setup.bash|g' "$BUILD_CONTEXT_DIR/docker/.bashrc"

# Copy the run script into build_context_release for easy deployment
cp "$REPO_ROOT/docker/run-jetson-release-install.sh" "$BUILD_CONTEXT_DIR/run.sh"
chmod +x "$BUILD_CONTEXT_DIR/run.sh"

echo ""
echo "Step 5: Building runtime Docker image..."

# Build the runtime image
cd "$BUILD_CONTEXT_DIR"
docker build -t "${IMAGE_NAME}:${IMAGE_TAG}" -f Dockerfile.runtime .

echo ""
echo "Step 6: Cleaning up..."
cd "$REPO_ROOT"
# Optional: uncomment to remove build context after successful build
# rm -rf "$BUILD_CONTEXT_DIR"

echo ""
echo "=== Build Complete ==="
echo "Runtime image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo ""
echo "Image details:"
docker images "${IMAGE_NAME}:${IMAGE_TAG}"
echo ""
echo "Build context preserved at: $BUILD_CONTEXT_DIR"
echo "  - Contains: install/, src/, docker/, Dockerfile.runtime, run.sh"
echo "  - Remove with: rm -rf $BUILD_CONTEXT_DIR"
echo ""
echo "To run the release build:"
echo "  cd $BUILD_CONTEXT_DIR && ./run.sh"
echo ""
echo "Or run directly:"
echo "  docker run -it --rm ${IMAGE_NAME}:${IMAGE_TAG}"
