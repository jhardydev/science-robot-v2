#!/bin/bash
# Simple script to run display test patterns
# Usage: ./test_display.sh

cd "$(dirname "$0")"

echo "Setting up ROS environment..."

# Try to source ROS setup if available
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    echo "Sourcing ROS Noetic..."
    source /opt/ros/noetic/setup.bash
elif [ -f "/opt/ros/melodic/setup.bash" ]; then
    echo "Sourcing ROS Melodic..."
    source /opt/ros/melodic/setup.bash
else
    echo "WARNING: ROS setup.bash not found in standard locations"
    echo "Please source ROS manually before running this script"
fi

# Try to source workspace if available
if [ -f "../devel/setup.bash" ]; then
    echo "Sourcing workspace devel/setup.bash..."
    source ../devel/setup.bash
elif [ -f "devel/setup.bash" ]; then
    echo "Sourcing workspace devel/setup.bash..."
    source devel/setup.bash
elif [ -f "~/catkin_ws/devel/setup.bash" ]; then
    echo "Sourcing catkin_ws..."
    source ~/catkin_ws/devel/setup.bash
fi

# Check if rospy is available
echo "Checking for rospy..."
python3 -c "import rospy" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "ERROR: rospy is not available!"
    echo ""
    echo "Please ensure ROS is properly installed and sourced:"
    echo "  source /opt/ros/noetic/setup.bash  # or melodic"
    echo "  source ~/catkin_ws/devel/setup.bash  # if using workspace"
    echo ""
    exit 1
fi

echo "ROS environment ready. Running test patterns..."
echo ""

# Run the test script directly with Python
python3 packages/science_robot/scripts/test_display_patterns.py "$@"

