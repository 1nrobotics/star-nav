#!/bin/bash

# Star Nav local development container (Linux host with X11 for RViz).
# Builds the image if needed and starts the `dev` compose service.

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR=$(dirname "$(realpath "$0")")
REPO_ROOT=${REPO_ROOT:-$(realpath "$SCRIPT_DIR/..")}
REBUILD_IMAGE=${REBUILD_IMAGE:-false}

cd "${REPO_ROOT}"

echo -e "${BLUE}Star Nav - local development container${NC}"

# Allow the container to open RViz windows on the host display.
xhost +local:docker 2>/dev/null || true

if [ "$REBUILD_IMAGE" = "true" ]; then
    echo "Rebuilding image..."
    docker compose build --no-cache dev
fi

docker compose up -d --build dev

echo ""
echo -e "${GREEN}Container is running.${NC}"
echo ""
echo "Useful commands:"
echo "  ./scripts/run_sim_exploration.sh   # Launch the simulation"
echo "  ./scripts/trigger_exploration.sh   # Trigger waypoint exploration"
echo "  ./scripts/run_smoke_test.sh        # Headless end-to-end test"
echo "  docker exec -it star_nav bash      # Shell into the container"
echo "  docker compose down                # Stop and remove the container"
