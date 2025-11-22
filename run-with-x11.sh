#!/bin/bash
# Helper script to run the robot with X11 display forwarding to macOS XQuartz

set -e

ROBOT_NAME=${VEHICLE_NAME:-robot1}
ROS_MASTER=${ROS_MASTER_URI:-http://localhost:11311}
LOG_DIR=${LOG_DIR:-/tmp/science-robot-logs}

# Check if DISPLAY is set (should be set by SSH X11 forwarding)
if [ -z "$DISPLAY" ]; then
    echo "ERROR: DISPLAY environment variable is not set."
    echo ""
    echo "X11 forwarding is required. Please:"
    echo "  1. Connect via SSH with X11 forwarding: ssh -X duckie@robot1"
    echo "  2. Or set DISPLAY manually: export DISPLAY=:10.0"
    echo ""
    echo "To verify X11 is working, try: xeyes"
    exit 1
fi

echo "Science Robot v2.0 - Running with X11 display forwarding"
echo "  Robot Name: $ROBOT_NAME"
echo "  ROS Master: $ROS_MASTER"
echo "  Display: $DISPLAY"
echo "  Log Directory: $LOG_DIR"
echo ""

# Verify X11 connection
if ! xdpyinfo > /dev/null 2>&1; then
    echo "WARNING: Cannot connect to X server at $DISPLAY"
    echo "  Make sure XQuartz is running on your Mac"
    echo "  And that you connected via SSH with X11 forwarding (ssh -X)"
    echo ""
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Create log directory on host
mkdir -p "$LOG_DIR"

# Determine X11 socket location
# For SSH X11 forwarding, DISPLAY is usually localhost:10.0
# We need to mount the X11 socket from the host
X11_SOCKET="/tmp/.X11-unix"

# Extract display number from DISPLAY (e.g., :10.0 -> 10)
DISPLAY_NUM=$(echo "$DISPLAY" | sed 's/.*:\([0-9]*\).*/\1/')

# For SSH X11 forwarding, the socket might be in a different location
# Try to find it
if [ -e "/tmp/.X11-unix/X${DISPLAY_NUM}" ]; then
    X11_SOCKET="/tmp/.X11-unix"
elif [ -e "/tmp/.X11-unix/X0" ]; then
    # Fall back to X0 if specific display not found
    X11_SOCKET="/tmp/.X11-unix"
    echo "Note: Using X0 socket (display number may differ)"
else
    echo "WARNING: X11 socket not found at /tmp/.X11-unix"
    echo "  X11 forwarding may not work correctly"
fi

# For SSH X11 forwarding, DISPLAY usually points to localhost
# Extract the host part
DISPLAY_HOST=$(echo "$DISPLAY" | cut -d: -f1)
if [ -z "$DISPLAY_HOST" ] || [ "$DISPLAY_HOST" = "$DISPLAY" ]; then
    # No host specified, assume localhost
    DISPLAY_HOST="localhost"
fi

# Build the full DISPLAY string for the container
# SSH X11 forwarding uses format like localhost:10.0
CONTAINER_DISPLAY="${DISPLAY_HOST}:${DISPLAY_NUM}.0"

# Find XAUTHORITY file (X11 authentication cookie)
# SSH X11 forwarding sets this, usually to ~/.Xauthority
XAUTHORITY_FILE="${XAUTHORITY:-$HOME/.Xauthority}"

if [ ! -f "$XAUTHORITY_FILE" ]; then
    echo "WARNING: XAUTHORITY file not found at $XAUTHORITY_FILE"
    echo "  X11 authentication may fail"
    echo "  Try: export XAUTHORITY=\$HOME/.Xauthority"
    XAUTHORITY_FILE=""
else
    echo "Found XAUTHORITY file: $XAUTHORITY_FILE"
fi

echo "X11 Configuration:"
echo "  Host DISPLAY: $DISPLAY"
echo "  Container DISPLAY: $CONTAINER_DISPLAY"
echo "  X11 Socket: $X11_SOCKET"
if [ -n "$XAUTHORITY_FILE" ]; then
    echo "  XAUTHORITY: $XAUTHORITY_FILE"
fi
echo ""

# Build volume mounts
VOLUME_MOUNTS=(
    -v "$X11_SOCKET:/tmp/.X11-unix:rw"
    -v "$LOG_DIR:/code/logs"
    -v /usr/lib/python3/dist-packages/vpi:/host/usr/lib/python3/dist-packages/vpi:ro
    -v /usr/lib/aarch64-linux-gnu:/host/usr/lib/aarch64-linux-gnu:ro
)

# Add XAUTHORITY mount if found
if [ -n "$XAUTHORITY_FILE" ] && [ -f "$XAUTHORITY_FILE" ]; then
    VOLUME_MOUNTS+=(-v "$XAUTHORITY_FILE:/root/.Xauthority:rw")
fi

# Build environment variables
ENV_VARS=(
    -e ROS_MASTER_URI="$ROS_MASTER"
    -e VEHICLE_NAME="$ROBOT_NAME"
    -e DISPLAY="$CONTAINER_DISPLAY"
    -e DISPLAY_OUTPUT=true
    -e LOG_DIR=/code/logs
)

# Add XAUTHORITY environment variable if found
if [ -n "$XAUTHORITY_FILE" ] && [ -f "$XAUTHORITY_FILE" ]; then
    ENV_VARS+=(-e XAUTHORITY=/root/.Xauthority)
fi

# Run container with X11 forwarding
docker run -it --rm --network host \
  "${ENV_VARS[@]}" \
  "${VOLUME_MOUNTS[@]}" \
  science-robot-v2:latest

echo ""
echo "Logs saved to: $LOG_DIR"
echo "  ROS logs: $LOG_DIR/ros/*/roslaunch-robot1-*.log"
echo "  Node logs: $LOG_DIR/ros/*/science_robot_controller-*.log"
echo "  Application logs: $LOG_DIR/*.log"

