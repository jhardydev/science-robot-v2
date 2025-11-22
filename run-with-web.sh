#!/bin/bash
# Helper script to run the robot with web server enabled

set -e

ROBOT_NAME=${VEHICLE_NAME:-robot1}
ROS_MASTER=${ROS_MASTER_URI:-http://localhost:11311}
LOG_DIR=${LOG_DIR:-/tmp/science-robot-logs}
WEB_PORT=${WEB_SERVER_PORT:-5000}

# Create log directory on host
mkdir -p "$LOG_DIR"

echo "Science Robot v2.0 - Running with Web Server"
echo "  Robot Name: $ROBOT_NAME"
echo "  ROS Master: $ROS_MASTER"
echo "  Log Directory: $LOG_DIR"
echo "  Web Server: http://0.0.0.0:$WEB_PORT"
echo ""

# Run container with web server enabled
docker run -it --rm --network host \
  -e ROS_MASTER_URI="$ROS_MASTER" \
  -e VEHICLE_NAME="$ROBOT_NAME" \
  -e LOG_DIR=/code/logs \
  -e ENABLE_WEB_SERVER=true \
  -e WEB_SERVER_PORT="$WEB_PORT" \
  -e DISPLAY_OUTPUT=false \
  -v "$LOG_DIR:/code/logs" \
  -v /usr/lib/python3/dist-packages/vpi:/host/usr/lib/python3/dist-packages/vpi:ro \
  -v /usr/lib/aarch64-linux-gnu:/host/usr/lib/aarch64-linux-gnu:ro \
  -p "$WEB_PORT:$WEB_PORT" \
  science-robot-v2:latest

echo ""
echo "Logs saved to: $LOG_DIR"
echo "Web dashboard was available at: http://robot1.local:$WEB_PORT"
echo "  (or http://<robot-ip>:$WEB_PORT from your Mac/iPad)"

