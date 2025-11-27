#!/usr/bin/env python3
"""
Run display test patterns
Usage: 
  python3 test_display_patterns.py
  OR
  rosrun science_robot test_display_patterns.py
"""
import sys
import os

# Try to import rospy - provide helpful error if not available
try:
    import rospy
except ImportError:
    print("ERROR: rospy module not found!")
    print("")
    print("Please ensure ROS is properly set up:")
    print("  1. Source ROS setup.bash:")
    print("     source /opt/ros/noetic/setup.bash  # For ROS Noetic")
    print("     # OR")
    print("     source /opt/ros/melodic/setup.bash  # For ROS Melodic")
    print("")
    print("  2. If using a workspace, source it:")
    print("     source ~/catkin_ws/devel/setup.bash")
    print("")
    print("  3. Then run this script again")
    sys.exit(1)

# Add package path - handle both direct execution and rosrun
script_dir = os.path.dirname(os.path.abspath(__file__))
# Try multiple possible paths
possible_paths = [
    os.path.join(script_dir, '../src'),  # From scripts/ directory
    os.path.join(script_dir, '../../src'),  # Alternative structure
    os.path.join(os.path.dirname(script_dir), 'src'),  # From package root
]

for package_src in possible_paths:
    if os.path.exists(package_src):
        sys.path.insert(0, package_src)
        break

from science_robot.display_test_pattern import DisplayTestPattern

if __name__ == "__main__":
    rospy.init_node('display_test_pattern_node', anonymous=True)
    
    tester = DisplayTestPattern()
    
    # Wait for publisher to connect
    rospy.loginfo("Waiting for display driver node to be available...")
    rospy.sleep(2)
    
    # Check if we should run all tests or single test
    if len(sys.argv) > 1 and sys.argv[1] == "--single":
        # Single test with custom parameters
        pattern_type = sys.argv[2] if len(sys.argv) > 2 else "grid"
        region = int(sys.argv[3]) if len(sys.argv) > 3 else 0
        x_offset = int(sys.argv[4]) if len(sys.argv) > 4 else 0
        y_offset = int(sys.argv[5]) if len(sys.argv) > 5 else 0
        width = int(sys.argv[6]) if len(sys.argv) > 6 else 128
        height = int(sys.argv[7]) if len(sys.argv) > 7 else 32
        
        rospy.loginfo(f"Running single test: {pattern_type}, region={region}, "
                     f"offset=({x_offset},{y_offset}), size={width}x{height}")
        tester.test_single_pattern(pattern_type, region, x_offset, y_offset, width, height)
    else:
        # Run all tests
        rospy.loginfo("Running all test patterns...")
        tester.test_all_regions()
    
    rospy.loginfo("Test patterns complete. Display should show test patterns.")
    rospy.loginfo("Take a photo and we can analyze the positioning.")
    rospy.loginfo("Press Ctrl+C to exit.")
    
    # Keep node alive
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Test pattern node shutting down...")

