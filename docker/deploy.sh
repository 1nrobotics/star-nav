#!/bin/bash

# Star Nav Docker Deployment Script
# For deploying to Radxa or other edge devices

set -e

# Colors for output  
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
IMAGE_NAME="1nrobotics/star-nav"
TAG=${1:-"latest"}
REMOTE_HOST=${2:-""}
DRONE_ID=${3:-"1"}

echo -e "${BLUE}🚁 Star Nav Deployment${NC}"

# Function to show usage
show_usage() {
    echo "Usage: $0 [TAG] [REMOTE_HOST] [DRONE_ID]"
    echo ""
    echo "Arguments:"
    echo "  TAG         Docker image tag (default: latest)"
    echo "  REMOTE_HOST Remote host (user@hostname) for deployment"
    echo "  DRONE_ID    Drone identifier (default: 1)"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Local deployment"
    echo "  $0 latest radxa@192.168.1.100 1      # Remote deployment to Radxa"
    echo "  $0 arm64 pi@drone.local 2            # Deploy to Raspberry Pi"
    echo ""
    echo "Modes:"
    echo "  --local         Deploy locally"
    echo "  --remote        Deploy to remote host"
    echo "  --simulation    Deploy with simulation"
    echo "  --development   Deploy development environment"
    echo "  --px4ctrl       Deploy PX4 controller only"
    echo "  --stop          Stop running containers"
    echo "  --logs          Show container logs"
    echo "  --status        Show deployment status"
    echo "  --setup-remote  Setup Docker on remote host"
}

# Function to deploy locally
deploy_local() {
    echo -e "${BLUE}🏠 Deploying locally...${NC}"
    
    # Stop existing containers
    docker-compose down 2>/dev/null || true
    
    # Pull latest image if not building locally
    if [[ "$TAG" != "latest" ]]; then
        docker pull "${IMAGE_NAME}:${TAG}"
    fi
    
    # Set environment variables
    export DRONE_ID="$DRONE_ID"
    export VEHICLE_ID="$DRONE_ID"
    
    # Start services
    docker-compose up -d
    
    echo -e "${GREEN}✅ Local deployment completed${NC}"
    
    # Show status
    docker-compose ps
}

# Function to deploy to remote host
deploy_remote() {
    if [[ -z "$REMOTE_HOST" ]]; then
        echo -e "${RED}❌ Remote host not specified${NC}"
        show_usage
        exit 1
    fi
    
    echo -e "${BLUE}🌐 Deploying to remote host: ${YELLOW}$REMOTE_HOST${NC}"
    
    # Check remote connectivity
    if ! ssh -o ConnectTimeout=10 "$REMOTE_HOST" "echo 'Connection successful'"; then
        echo -e "${RED}❌ Cannot connect to remote host${NC}"
        exit 1
    fi
    
    # Copy deployment files
    echo -e "${BLUE}📤 Copying project files...${NC}"
    rsync -avz \
        --exclude='.git' \
        --exclude='build' \
        --exclude='devel' \
        --exclude='logs' \
        --exclude='*.bag' \
        --progress \
        . "$REMOTE_HOST:~/star_nav/"
    
    # Build and deploy on remote device
    echo -e "${BLUE}🔨 Building on remote device...${NC}"
    ssh "$REMOTE_HOST" "
        cd ~/star_nav
        echo 'Building Docker image on device...'
        docker build -t 1nrobotics/star-nav:local .
        
        echo 'Starting deployment...'
        export DRONE_ID=$DRONE_ID
        export VEHICLE_ID=$DRONE_ID
        docker-compose down 2>/dev/null || true
        docker-compose up -d
        
        echo 'Deployment completed!'
        docker-compose ps
    "
    
    echo -e "${GREEN}✅ Remote deployment completed${NC}"
}

# Function to deploy with simulation
deploy_simulation() {
    echo -e "${BLUE}🎮 Deploying with simulation...${NC}"
    
    export DRONE_ID="$DRONE_ID"
    export VEHICLE_ID="$DRONE_ID"
    
    # Enable X11 forwarding for GUI
    xhost +local:docker 2>/dev/null || true
    
    # Start simulation environment
    docker-compose --profile simulation up -d
    
    echo -e "${GREEN}✅ Simulation deployment completed${NC}"
    docker-compose ps
}

# Function to deploy development environment
deploy_development() {
    echo -e "${BLUE}🛠️ Deploying development environment...${NC}"
    
    export DRONE_ID="$DRONE_ID"
    export VEHICLE_ID="$DRONE_ID"
    
    # Enable X11 forwarding for GUI
    xhost +local:docker 2>/dev/null || true
    
    # Start development environment
    docker-compose --profile development up -d
    
    echo -e "${GREEN}✅ Development deployment completed${NC}"
    
    echo ""
    echo -e "${BLUE}Access development container:${NC}"
    echo "  docker-compose exec star_nav_dev bash"
}

# Function to deploy PX4 controller only
deploy_px4ctrl() {
    echo -e "${BLUE}🎮 Deploying PX4 Controller...${NC}"
    
    export DRONE_ID="$DRONE_ID"
    export VEHICLE_ID="$DRONE_ID"
    
    # Start PX4 controller
    docker-compose --profile px4ctrl up -d
    
    echo -e "${GREEN}✅ PX4 Controller deployment completed${NC}"
    docker-compose ps
}

# Function to stop deployment
stop_deployment() {
    echo -e "${BLUE}🛑 Stopping deployment...${NC}"
    
    if [[ -n "$REMOTE_HOST" ]]; then
        ssh "$REMOTE_HOST" "cd ~/star_nav && docker-compose down"
    else
        docker-compose down
    fi
    
    echo -e "${GREEN}✅ Deployment stopped${NC}"
}

# Function to show logs
show_logs() {
    echo -e "${BLUE}📋 Showing container logs...${NC}"
    
    if [[ -n "$REMOTE_HOST" ]]; then
        ssh "$REMOTE_HOST" "cd ~/star_nav && docker-compose logs -f"
    else
        docker-compose logs -f
    fi
}

# Function to show status
show_status() {
    echo -e "${BLUE}📊 Deployment Status${NC}"
    
    if [[ -n "$REMOTE_HOST" ]]; then
        echo -e "Remote Host: ${YELLOW}$REMOTE_HOST${NC}"
        ssh "$REMOTE_HOST" "cd ~/star_nav && docker-compose ps"
    else
        echo -e "Local Deployment"
        docker-compose ps 2>/dev/null || echo "No containers running"
    fi
}

# Function to setup remote host
setup_remote_host() {
    # For setup mode, REMOTE_HOST should be the first argument after --setup-remote
    local setup_host=${2:-$REMOTE_HOST}
    if [[ -z "$setup_host" ]]; then
        echo -e "${RED}❌ Remote host not specified${NC}"
        echo "Usage: $0 --setup-remote [REMOTE_HOST]"
        exit 1
    fi
    REMOTE_HOST=$setup_host
    
    echo -e "${BLUE}⚙️  Setting up remote host: ${YELLOW}$REMOTE_HOST${NC}"
    
    # Install Docker if not present
    ssh "$REMOTE_HOST" "
        if ! command -v docker &> /dev/null; then
            echo 'Installing Docker...'
            curl -fsSL https://get.docker.com -o get-docker.sh
            sudo sh get-docker.sh
            sudo usermod -aG docker \$USER
            echo 'Docker installed. Please log out and back in.'
        fi
        
        if ! command -v docker-compose &> /dev/null; then
            echo 'Installing Docker Compose...'
            sudo apt update
            sudo apt install docker-compose -y
        fi
        
        echo 'Remote host setup completed'
    "
    
    echo -e "${GREEN}✅ Remote host setup completed${NC}"
}

# Parse command line arguments
MODE="local"
while [[ $# -gt 0 ]]; do
    case $1 in
        --local)
            MODE="local"
            shift
            ;;
        --remote)
            MODE="remote"
            shift
            ;;
        --simulation)
            MODE="simulation"
            shift
            ;;
        --development)
            MODE="development"
            shift
            ;;
        --px4ctrl)
            MODE="px4ctrl"
            shift
            ;;
        --stop)
            MODE="stop"
            shift
            ;;
        --logs)
            MODE="logs"
            shift
            ;;
        --status)
            MODE="status"
            shift
            ;;
        --setup-remote)
            MODE="setup"
            shift
            ;;
        --help|-h)
            show_usage
            exit 0
            ;;
        *)
            break
            ;;
    esac
done

# Main execution
main() {
    case $MODE in
        "local")
            deploy_local
            ;;
        "remote")
            deploy_remote
            ;;
        "simulation")
            deploy_simulation
            ;;
        "development")
            deploy_development
            ;;
        "px4ctrl")
            deploy_px4ctrl
            ;;
        "stop")
            stop_deployment
            ;;
        "logs")
            show_logs
            ;;
        "status")
            show_status
            ;;
        "setup")
            setup_remote_host
            ;;
        *)
            echo -e "${RED}❌ Unknown mode: $MODE${NC}"
            show_usage
            exit 1
            ;;
    esac
}

# Check prerequisites
check_prerequisites() {
    if [[ "$MODE" != "setup" ]] && [[ "$MODE" != "remote" ]]; then
        if ! command -v docker &> /dev/null; then
            echo -e "${RED}❌ Docker is not installed${NC}"
            exit 1
        fi
        
        if ! command -v docker-compose &> /dev/null; then
            echo -e "${RED}❌ Docker Compose is not installed${NC}"
            exit 1
        fi
    fi
}

# Run main function
check_prerequisites
main "$@"
