#!/bin/bash
# Test script for collision avoidance and enhanced emergency stop
# Tests the new safety features

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

IMAGE_NAME="science-robot-v2:latest"
ROBOT_NAME=${VEHICLE_NAME:-robot1}
ROS_MASTER=${ROS_MASTER_URI:-http://localhost:11311}

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Testing Collision Avoidance & Emergency Stop${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

# Check if image exists
if ! docker image inspect "$IMAGE_NAME" > /dev/null 2>&1; then
    echo -e "${RED}✗ Docker image not found: $IMAGE_NAME${NC}"
    echo -e "${YELLOW}  Building image first...${NC}"
    docker build -t "$IMAGE_NAME" .
    echo -e "${GREEN}✓ Image built${NC}"
fi

echo ""
echo -e "${BLUE}Test 1: Syntax Check${NC}"
echo "Checking Python syntax for new modules..."

docker run --rm \
    -v "$(pwd)/packages:/code/packages:ro" \
    "$IMAGE_NAME" \
    python3 -m py_compile \
        /code/packages/science_robot/src/science_robot/collision_avoidance.py \
        /code/packages/science_robot/src/science_robot/motor_controller.py \
        /code/packages/science_robot/src/science_robot/config.py

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Syntax check passed${NC}"
else
    echo -e "${RED}✗ Syntax errors found${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}Test 2: Import Test${NC}"
echo "Testing if modules can be imported..."

docker run --rm \
    -e "ROS_MASTER_URI=${ROS_MASTER}" \
    -e "VEHICLE_NAME=${ROBOT_NAME}" \
    -v "$(pwd)/packages:/code/packages:ro" \
    "$IMAGE_NAME" \
    bash -c "
        source /opt/ros/noetic/setup.bash
        export PYTHONPATH=/code/packages/src:\$PYTHONPATH
        python3 -c '
import sys
sys.path.insert(0, \"/code/packages/science_robot/src\")
try:
    from science_robot import config
    print(\"✓ config imported\")
    
    from science_robot.collision_avoidance import CollisionAvoidance
    print(\"✓ CollisionAvoidance imported\")
    
    from science_robot.motor_controller import MotorController
    print(\"✓ MotorController imported\")
    
    # Check config values
    print(f\"  Collision avoidance enabled: {config.ENABLE_COLLISION_AVOIDANCE}\")
    print(f\"  Emergency distance: {config.COLLISION_EMERGENCY_DISTANCE}m\")
    print(f\"  Warning distance: {config.COLLISION_WARNING_DISTANCE}m\")
    print(f\"  Safe distance: {config.COLLISION_SAFE_DISTANCE}m\")
    
    print(\"✓ All imports successful\")
except Exception as e:
    print(f\"✗ Import error: {e}\")
    import traceback
    traceback.print_exc()
    sys.exit(1)
'
    "

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Import test passed${NC}"
else
    echo -e "${RED}✗ Import test failed${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}Test 3: Configuration Validation${NC}"
echo "Checking collision avoidance configuration..."

docker run --rm \
    -e "ROS_MASTER_URI=${ROS_MASTER}" \
    -e "VEHICLE_NAME=${ROBOT_NAME}" \
    -e "ENABLE_COLLISION_AVOIDANCE=True" \
    -e "COLLISION_EMERGENCY_DISTANCE=0.15" \
    -e "COLLISION_WARNING_DISTANCE=0.30" \
    -e "COLLISION_SAFE_DISTANCE=0.50" \
    -v "$(pwd)/packages:/code/packages:ro" \
    "$IMAGE_NAME" \
    bash -c "
        source /opt/ros/noetic/setup.bash
        export PYTHONPATH=/code/packages/src:\$PYTHONPATH
        python3 -c '
import sys
sys.path.insert(0, \"/code/packages/science_robot/src\")
from science_robot import config

assert config.ENABLE_COLLISION_AVOIDANCE == True, \"Collision avoidance should be enabled\"
assert config.COLLISION_EMERGENCY_DISTANCE == 0.15, \"Emergency distance should be 0.15m\"
assert config.COLLISION_WARNING_DISTANCE == 0.30, \"Warning distance should be 0.30m\"
assert config.COLLISION_SAFE_DISTANCE == 0.50, \"Safe distance should be 0.50m\"
assert config.COLLISION_EMERGENCY_DISTANCE < config.COLLISION_WARNING_DISTANCE, \"Emergency should be less than warning\"
assert config.COLLISION_WARNING_DISTANCE < config.COLLISION_SAFE_DISTANCE, \"Warning should be less than safe\"

print(\"✓ Configuration validation passed\")
print(f\"  Emergency zone: {config.COLLISION_EMERGENCY_DISTANCE}m\")
print(f\"  Warning zone: {config.COLLISION_WARNING_DISTANCE}m\")
print(f\"  Safe zone: {config.COLLISION_SAFE_DISTANCE}m\")
'
    "

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Configuration validation passed${NC}"
else
    echo -e "${RED}✗ Configuration validation failed${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}Test 4: Emergency Stop Topic Check${NC}"
echo "Verifying emergency stop topic configuration..."

docker run --rm \
    -e "ROS_MASTER_URI=${ROS_MASTER}" \
    -e "VEHICLE_NAME=${ROBOT_NAME}" \
    -v "$(pwd)/packages:/code/packages:ro" \
    "$IMAGE_NAME" \
    bash -c "
        source /opt/ros/noetic/setup.bash
        export PYTHONPATH=/code/packages/src:\$PYTHONPATH
        python3 -c '
import sys
sys.path.insert(0, \"/code/packages/science_robot/src\")
from science_robot import config

expected_topic = f\"/{config.ROBOT_NAME}/wheels_driver_node/emergency_stop\"
actual_topic = config.EMERGENCY_STOP_TOPIC

assert actual_topic == expected_topic, f\"Emergency stop topic mismatch: {actual_topic} != {expected_topic}\"

print(f\"✓ Emergency stop topic configured: {actual_topic}\")
'
    "

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Emergency stop topic check passed${NC}"
else
    echo -e "${RED}✗ Emergency stop topic check failed${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}Test 5: Collision Avoidance Module Structure${NC}"
echo "Checking collision avoidance class methods..."

docker run --rm \
    -e "ROS_MASTER_URI=${ROS_MASTER}" \
    -e "VEHICLE_NAME=${ROBOT_NAME}" \
    -v "$(pwd)/packages:/code/packages:ro" \
    "$IMAGE_NAME" \
    bash -c "
        source /opt/ros/noetic/setup.bash
        export PYTHONPATH=/code/packages/src:\$PYTHONPATH
        python3 -c '
import sys
sys.path.insert(0, \"/code/packages/science_robot/src\")
from science_robot.collision_avoidance import CollisionAvoidance
import inspect

# Check required methods exist
required_methods = [
    \"check_collision_risk\",
    \"detect_obstacles_video\",
    \"get_safe_speed\",
    \"draw_overlay\",
    \"cleanup\"
]

for method in required_methods:
    assert hasattr(CollisionAvoidance, method), f\"Missing method: {method}\"
    print(f\"  ✓ {method}() exists\")

print(\"✓ All required methods present\")
'
    "

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Module structure check passed${NC}"
else
    echo -e "${RED}✗ Module structure check failed${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}Test 6: Motor Controller Emergency Stop Methods${NC}"
echo "Checking enhanced emergency stop functionality..."

docker run --rm \
    -e "ROS_MASTER_URI=${ROS_MASTER}" \
    -e "VEHICLE_NAME=${ROBOT_NAME}" \
    -v "$(pwd)/packages:/code/packages:ro" \
    "$IMAGE_NAME" \
    bash -c "
        source /opt/ros/noetic/setup.bash
        export PYTHONPATH=/code/packages/src:\$PYTHONPATH
        python3 -c '
import sys
sys.path.insert(0, \"/code/packages/science_robot/src\")
from science_robot.motor_controller import MotorController
import inspect

# Check required methods exist
required_methods = [
    \"emergency_stop\",
    \"clear_emergency_stop\",
    \"is_emergency_stop_active\",
    \"get_last_speeds\"
]

for method in required_methods:
    assert hasattr(MotorController, method), f\"Missing method: {method}\"
    print(f\"  ✓ {method}() exists\")

print(\"✓ All emergency stop methods present\")
'
    "

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Motor controller check passed${NC}"
else
    echo -e "${RED}✗ Motor controller check failed${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}All basic tests passed!${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "${YELLOW}Next steps for runtime testing:${NC}"
echo "1. Build/rebuild Docker image if needed:"
echo "   docker build -t $IMAGE_NAME ."
echo ""
echo "2. Test emergency stop via web interface:"
echo "   ./run-with-web.sh"
echo "   Then click 'Emergency Stop' button"
echo ""
echo "3. Test collision avoidance (requires ROS master and camera):"
echo "   ./run-with-web.sh"
echo "   Monitor collision risk status in web dashboard"
echo ""
echo "4. Test with ToF sensors (if available):"
echo "   Ensure ToF sensor topics are published, then run:"
echo "   ./run-with-web.sh"
echo "   Collision avoidance will use ToF data if available"
echo ""

