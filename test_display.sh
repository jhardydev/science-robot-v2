#!/bin/bash
# Simple script to run display test patterns
# Usage: ./test_display.sh

cd "$(dirname "$0")"

# Try to source ROS setup if available
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f "/opt/ros/melodic/setup.bash" ]; then
    source /opt/ros/melodic/setup.bash
fi

# Try to source workspace if available
if [ -f "../devel/setup.bash" ]; then
    source ../devel/setup.bash
elif [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
fi

# Run the test script directly with Python
python3 packages/science_robot/scripts/test_display_patterns.py "$@"

