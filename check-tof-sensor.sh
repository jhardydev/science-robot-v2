#!/bin/bash
# Helper script to check ToF sensor status
# Can be run from host or inside container

set -e

ROBOT_NAME=${VEHICLE_NAME:-robot1}
ROS_MASTER=${ROS_MASTER_URI:-http://localhost:11311}

echo "ToF Sensor Diagnostic Tool"
echo "============================"
echo "Robot Name: $ROBOT_NAME"
echo "ROS Master: $ROS_MASTER"
echo ""

# Check if we're in a container or on host
if [ -f /.dockerenv ] || [ -n "$DOCKER_CONTAINER" ]; then
    echo "Running inside container"
    CONTAINER_MODE=true
else
    echo "Running on host - will use docker exec"
    CONTAINER_MODE=false
    
    # Find running container
    CONTAINER_NAME=$(docker ps --filter "ancestor=science-robot-v2:latest" --format "{{.Names}}" | head -1)
    if [ -z "$CONTAINER_NAME" ]; then
        echo "Error: No running science-robot-v2 container found"
        echo "Please start the robot first: ./run-with-web.sh"
        exit 1
    fi
    echo "Found container: $CONTAINER_NAME"
fi

check_tof() {
    echo ""
    echo "1. Checking ROS master connection..."
    if rostopic list > /dev/null 2>&1; then
        echo "   ✓ ROS master is accessible"
    else
        echo "   ✗ Cannot connect to ROS master at $ROS_MASTER"
        echo "   Make sure ROS master is running"
        return 1
    fi
    
    echo ""
    echo "2. Checking for ToF-related topics..."
    TOF_TOPICS=$(rostopic list 2>/dev/null | grep -iE "tof|distance|range|0x29|vl53" || true)
    if [ -n "$TOF_TOPICS" ]; then
        echo "   ✓ Found ToF-related topics:"
        echo "$TOF_TOPICS" | sed 's/^/      - /'
    else
        echo "   ✗ No ToF-related topics found"
        echo "   Available topics:"
        rostopic list 2>/dev/null | head -10 | sed 's/^/      /'
    fi
    
    echo ""
    echo "3. Checking topic message types..."
    for topic in $(rostopic list 2>/dev/null | grep -iE "tof|distance|range" | head -5); do
        msg_type=$(rostopic type "$topic" 2>/dev/null || echo "unknown")
        echo "   $topic -> $msg_type"
    done
    
    echo ""
    echo "4. Checking if ToF topics are publishing..."
    TOF_TOPIC=$(rostopic list 2>/dev/null | grep -iE "/tof.*distance|/tof.*range" | head -1)
    if [ -n "$TOF_TOPIC" ]; then
        echo "   Testing topic: $TOF_TOPIC"
        echo "   Publishing frequency:"
        timeout 3 rostopic hz "$TOF_TOPIC" 2>&1 | head -3 || echo "   ✗ Topic not publishing"
        
        echo ""
        echo "   Latest distance reading:"
        timeout 2 rostopic echo "$TOF_TOPIC" -n 1 2>&1 | grep -E "range|distance|data" | head -3 || echo "   ✗ No data received"
    else
        echo "   ✗ No ToF distance/range topics found"
    fi
    
    echo ""
    echo "5. Checking for ToF nodes..."
    TOF_NODES=$(rosnode list 2>/dev/null | grep -i tof || true)
    if [ -n "$TOF_NODES" ]; then
        echo "   ✓ Found ToF nodes:"
        echo "$TOF_NODES" | sed 's/^/      - /'
    else
        echo "   ✗ No ToF nodes found"
        echo "   You may need to start the ToF sensor node:"
        echo "   roslaunch duckietown tof_node.launch veh:=$ROBOT_NAME"
    fi
}

if [ "$CONTAINER_MODE" = true ]; then
    # Source ROS environment
    source /opt/ros/noetic/setup.bash 2>/dev/null || true
    source /code/packages/devel/setup.bash 2>/dev/null || true
    
    # Set ROS environment
    export ROS_MASTER_URI=$ROS_MASTER
    export VEHICLE_NAME=$ROBOT_NAME
    
    check_tof
else
    # Run inside container
    docker exec -it "$CONTAINER_NAME" bash -c "
        source /opt/ros/noetic/setup.bash
        source /code/packages/devel/setup.bash 2>/dev/null || true
        export ROS_MASTER_URI=$ROS_MASTER
        export VEHICLE_NAME=$ROBOT_NAME
        $(declare -f check_tof)
        check_tof
    "
fi

echo ""
echo "============================"
echo "Diagnostic complete"
echo ""
echo "If ToF sensor is not found:"
echo "  1. Check if ToF node is running: rosnode list | grep tof"
echo "  2. Start ToF node if needed: roslaunch duckietown tof_node.launch veh:=$ROBOT_NAME"
echo "  3. Check I2C connection: i2cdetect -y 1 (should show 0x29)"
echo "  4. Use manual override: export TOF_TOPIC_OVERRIDE=\"/your/topic/name\""

