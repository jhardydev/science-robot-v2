#!/bin/bash
# Manual test for Test 5 - ROS Node Startup
# Run this to see what happens when we try to start the node

echo "=== Manual Test 5: ROS Node Startup ==="
echo ""

# Configuration
IMAGE_NAME="science-robot-v2:latest"
ROBOT_NAME="robot1"
ROS_MASTER="http://localhost:11311"

echo "Configuration:"
echo "  Image: ${IMAGE_NAME}"
echo "  Robot Name: ${ROBOT_NAME}"
echo "  ROS Master: ${ROS_MASTER}"
echo ""

echo "Running docker command (will timeout after 10 seconds)..."
echo "Command:"
echo "  timeout 20 docker run --rm --network host \\"
echo "    -e ROS_MASTER_URI=${ROS_MASTER} \\"
echo "    -e VEHICLE_NAME=${ROBOT_NAME} \\"
echo "    -v /usr/lib/python3/dist-packages/vpi:/host/usr/lib/python3/dist-packages/vpi:ro \\"
echo "    -v /usr/lib/aarch64-linux-gnu:/host/usr/lib/aarch64-linux-gnu:ro \\"
echo "    ${IMAGE_NAME} \\"
echo "    timeout 10 roslaunch science_robot science_robot.launch robot_name:=${ROBOT_NAME}"
echo ""
echo "--- Output ---"

# Run the command without redirecting to see output immediately
timeout 20 docker run --rm --network host \
  -e "ROS_MASTER_URI=${ROS_MASTER}" \
  -e "VEHICLE_NAME=${ROBOT_NAME}" \
  -v "/usr/lib/python3/dist-packages/vpi:/host/usr/lib/python3/dist-packages/vpi:ro" \
  -v "/usr/lib/aarch64-linux-gnu:/host/usr/lib/aarch64-linux-gnu:ro" \
  "${IMAGE_NAME}" \
  timeout 10 roslaunch science_robot science_robot.launch robot_name:="${ROBOT_NAME}"

EXIT_CODE=$?

echo ""
echo "--- End of Output ---"
echo "Exit code: $EXIT_CODE"

