#!/bin/bash
# Helper script to run the robot with logs saved to host /tmp

set -e

ROBOT_NAME=${VEHICLE_NAME:-robot1}
ROS_MASTER=${ROS_MASTER_URI:-http://localhost:11311}
LOG_DIR=${LOG_DIR:-/tmp/science-robot-logs}

# Create log directory on host
mkdir -p "$LOG_DIR"

echo "Science Robot v2.0 - Running with host log directory"
echo "  Robot Name: $ROBOT_NAME"
echo "  ROS Master: $ROS_MASTER"
echo "  Log Directory: $LOG_DIR"
echo ""

# Run container with log directory mounted
docker run -it --rm --network host \
  -e ROS_MASTER_URI="$ROS_MASTER" \
  -e VEHICLE_NAME="$ROBOT_NAME" \
  -e LOG_DIR=/code/logs \
  -v "$LOG_DIR:/code/logs" \
  -v /usr/lib/python3/dist-packages/vpi:/host/usr/lib/python3/dist-packages/vpi:ro \
  -v /usr/lib/aarch64-linux-gnu:/host/usr/lib/aarch64-linux-gnu:ro \
  science-robot-v2:latest

echo ""
echo "Logs saved to: $LOG_DIR"
echo "  ROS logs: $LOG_DIR/ros/"
echo "  Application logs: $LOG_DIR/"

