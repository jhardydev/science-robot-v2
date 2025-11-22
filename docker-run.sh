#!/bin/bash
# Helper script to build and run the Duckiebot Science Fair Robot container

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default values
ROBOT_NAME=${VEHICLE_NAME:-robot1}
ROS_MASTER=${ROS_MASTER_URI:-http://localhost:11311}
BUILD=false
USE_VIRTUAL_DISPLAY=${USE_VIRTUAL_DISPLAY:-false}
DISPLAY_OUTPUT=${DISPLAY_OUTPUT:-false}
ENABLE_VNC=${ENABLE_VNC:-false}
VNC_PORT=${VNC_PORT:-5900}
LOG_DIR="${HOME}/science-robot-logs"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --robot-name)
            ROBOT_NAME="$2"
            shift 2
            ;;
        --ros-master)
            ROS_MASTER="$2"
            shift 2
            ;;
        --build)
            BUILD=true
            shift
            ;;
        --virtual-display)
            USE_VIRTUAL_DISPLAY=true
            DISPLAY_OUTPUT=true
            shift
            ;;
        --display-output)
            DISPLAY_OUTPUT=true
            shift
            ;;
        --vnc)
            USE_VIRTUAL_DISPLAY=true
            DISPLAY_OUTPUT=true
            ENABLE_VNC=true
            shift
            ;;
        --vnc-port)
            VNC_PORT="$2"
            shift 2
            ;;
        --log-dir)
            LOG_DIR="$2"
            shift 2
            ;;
        --cleanup)
            # Run cleanup script
            SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
            "$SCRIPT_DIR/docker-cleanup.sh" --all --force
            exit 0
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --robot-name NAME       Set robot name (default: robot1)"
            echo "  --ros-master URI        Set ROS master URI (default: http://localhost:11311)"
            echo "  --build                 Build image before running"
            echo "  --virtual-display       Enable virtual display (Xvfb) for headless video output"
            echo "  --display-output        Enable display output (requires X11 or virtual display)"
            echo "  --vnc                   Enable virtual display with VNC server (combines --virtual-display + VNC)"
            echo "  --vnc-port PORT         Set VNC port (default: 5900)"
            echo "  --log-dir DIR           Set log directory (default: ~/science-robot-logs)"
            echo "  --cleanup               Clean up containers, images, and volumes (then exit)"
            echo "  --help                  Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0 --build"
            echo "  $0 --robot-name duckiebot --ros-master http://192.168.1.100:11311"
            echo "  $0 --cleanup          # Clean up everything before running"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo -e "${GREEN}Duckiebot Science Fair Robot - Docker Runner${NC}"
echo "Robot name: $ROBOT_NAME"
echo "ROS Master: $ROS_MASTER"
echo "Virtual display: $USE_VIRTUAL_DISPLAY"
echo "Display output: $DISPLAY_OUTPUT"
if [ "$USE_VIRTUAL_DISPLAY" = "true" ]; then
    echo "VNC enabled: $ENABLE_VNC"
    if [ "$ENABLE_VNC" = "true" ]; then
        echo "VNC port: $VNC_PORT"
    fi
fi
echo "Log directory: $LOG_DIR"
echo ""

# Create log directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Build if requested
if [ "$BUILD" = true ]; then
    echo -e "${YELLOW}Building Docker image...${NC}"
    docker build -t science-robot:latest .
    echo ""
fi

# Check if image exists
if ! docker image inspect science-robot:latest > /dev/null 2>&1; then
    echo -e "${YELLOW}Image not found. Building...${NC}"
    docker build -t science-robot:latest .
    echo ""
fi

# Stop existing container if running
if docker ps -a --format '{{.Names}}' | grep -q "^science-robot$"; then
    echo -e "${YELLOW}Stopping existing container...${NC}"
    docker stop science-robot > /dev/null 2>&1 || true
    docker rm science-robot > /dev/null 2>&1 || true
    echo ""
fi

# Run container
echo -e "${GREEN}Starting container...${NC}"

# Build volume mounts
VOLUME_MOUNTS=(
    -v "$LOG_DIR:/code/logs"
)

# Mount VPI libraries from host (if available on Jetson)
# VPI is typically installed as part of JetPack SDK
# Only mount VPI package specifically, not entire dist-packages (avoids numpy conflicts)
VPI_MOUNTED=false

# Method 1: Try to find VPI via Python import (most reliable)
VPI_PYTHON_PATH=$(python3 -c "import vpi; import os; print(os.path.dirname(vpi.__file__))" 2>/dev/null || echo "")
if [ -n "$VPI_PYTHON_PATH" ] && [ -d "$VPI_PYTHON_PATH" ]; then
    echo "Found VPI Python package via import: $VPI_PYTHON_PATH"
    # Mount only the VPI package directory, not parent dist-packages
    VOLUME_MOUNTS+=(-v "$VPI_PYTHON_PATH:/host$VPI_PYTHON_PATH:ro")
    echo "Mounting VPI Python package: $VPI_PYTHON_PATH -> /host$VPI_PYTHON_PATH"
    VPI_MOUNTED=true
    
    # Also need parent directory in PYTHONPATH so Python can find it
    # We'll add this to PYTHONPATH in entrypoint, not mount entire dist-packages
    VPI_PARENT=$(dirname "$VPI_PYTHON_PATH" 2>/dev/null || echo "")
    if [ -n "$VPI_PARENT" ]; then
        # Export parent path for entrypoint to use
        export VPI_PARENT_DIR="$VPI_PARENT"
        echo "VPI parent directory for PYTHONPATH: $VPI_PARENT"
    fi
else
    echo "VPI not found via Python import - trying file system search..."
fi

# Method 2: Try common installation paths (if import failed)
if [ "$VPI_MOUNTED" = false ]; then
    # Try to mount only the vpi package, not entire dist-packages
    if [ -d "/usr/lib/python3/dist-packages/vpi" ]; then
        VOLUME_MOUNTS+=(-v "/usr/lib/python3/dist-packages/vpi:/host/usr/lib/python3/dist-packages/vpi:ro")
        echo "Mounting VPI package from: /usr/lib/python3/dist-packages/vpi"
        VPI_MOUNTED=true
    elif [ -f "/usr/lib/python3/dist-packages/vpi.py" ]; then
        # VPI might be a single file module
        VOLUME_MOUNTS+=(-v "/usr/lib/python3/dist-packages/vpi.py:/host/usr/lib/python3/dist-packages/vpi.py:ro")
        echo "Mounting VPI module from: /usr/lib/python3/dist-packages/vpi.py"
        VPI_MOUNTED=true
    fi
fi

# Mount VPI shared libraries (needed for VPI to work)
if [ -d "/usr/lib/aarch64-linux-gnu" ]; then
    # Check if VPI libraries exist
    if ls /usr/lib/aarch64-linux-gnu/libnvvpi* > /dev/null 2>&1; then
        VOLUME_MOUNTS+=(-v "/usr/lib/aarch64-linux-gnu:/host/usr/lib/aarch64-linux-gnu:ro")
        echo "Mounting aarch64 libraries from: /usr/lib/aarch64-linux-gnu"
    fi
fi

# Add X11 mounts if not using virtual display
if [ "$USE_VIRTUAL_DISPLAY" != "true" ] && [ -n "$DISPLAY" ] && [ -e /tmp/.X11-unix ]; then
    VOLUME_MOUNTS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
    if [ -e "$HOME/.Xauthority" ]; then
        VOLUME_MOUNTS+=(-v "$HOME/.Xauthority:/root/.Xauthority:rw")
    fi
fi

# Build environment variables
ENV_VARS=(
    -e ROS_MASTER_URI="$ROS_MASTER"
    -e ROS_HOSTNAME=science-robot
    -e VEHICLE_NAME="$ROBOT_NAME"
    -e USE_VIRTUAL_DISPLAY="$USE_VIRTUAL_DISPLAY"
    -e DISPLAY_OUTPUT="$DISPLAY_OUTPUT"
    -e ENABLE_VNC="$ENABLE_VNC"
    -e VNC_PORT="$VNC_PORT"
    # Pass through NVIDIA runtime for GPU access
    -e NVIDIA_VISIBLE_DEVICES=all
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility
)

# Add VPI_PARENT_DIR if it was set
if [ -n "$VPI_PARENT_DIR" ]; then
    ENV_VARS+=(-e VPI_PARENT_DIR="$VPI_PARENT_DIR")
fi

# Add DISPLAY if not using virtual display
if [ "$USE_VIRTUAL_DISPLAY" != "true" ] && [ -n "$DISPLAY" ]; then
    ENV_VARS+=(-e DISPLAY="$DISPLAY")
fi

# Add VNC port mapping if VNC is enabled
PORT_MAPPINGS=()
if [ "$ENABLE_VNC" = "true" ]; then
    PORT_MAPPINGS=(-p "${VNC_PORT}:${VNC_PORT}")
    echo "VNC will be accessible on port ${VNC_PORT}"
    echo "  Direct: robot-ip:${VNC_PORT}"
    echo "  SSH tunnel: ssh -L ${VNC_PORT}:localhost:${VNC_PORT} user@robot-ip"
fi

# Check for GPU support (NVIDIA runtime)
GPU_FLAG=""
if command -v nvidia-smi > /dev/null 2>&1; then
    # Try --gpus flag first (newer Docker)
    if docker run --help 2>&1 | grep -q "\-\-gpus"; then
        GPU_FLAG="--gpus all"
        echo "Using --gpus all for GPU access"
    # Fall back to --runtime=nvidia (older Docker)
    elif docker info 2>&1 | grep -q "nvidia"; then
        GPU_FLAG="--runtime=nvidia"
        echo "Using --runtime=nvidia for GPU access"
    fi
fi

docker run -it --rm \
    --name science-robot \
    --network host \
    $GPU_FLAG \
    "${PORT_MAPPINGS[@]}" \
    "${ENV_VARS[@]}" \
    "${VOLUME_MOUNTS[@]}" \
    science-robot:latest

