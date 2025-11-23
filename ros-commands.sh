#!/bin/bash
# Helper script to run ROS commands inside the container
# Usage: ./ros-commands.sh <ros-command>
# Example: ./ros-commands.sh "rostopic list"

set -e

# Find running container
CONTAINER_NAME=$(docker ps --filter "ancestor=science-robot-v2:latest" --format "{{.Names}}" | head -1)

if [ -z "$CONTAINER_NAME" ]; then
    echo "Error: No running science-robot-v2 container found"
    echo "Please start the robot first: ./run-with-web.sh"
    exit 1
fi

ROBOT_NAME=${VEHICLE_NAME:-robot1}
ROS_MASTER=${ROS_MASTER_URI:-http://localhost:11311}

# Run ROS command inside container
docker exec -it "$CONTAINER_NAME" bash -c "
    source /opt/ros/noetic/setup.bash
    source /code/packages/devel/setup.bash 2>/dev/null || true
    export ROS_MASTER_URI=$ROS_MASTER
    export VEHICLE_NAME=$ROBOT_NAME
    $*
"

