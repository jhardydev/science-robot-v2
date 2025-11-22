cmake_minimum_required(VERSION 3.0.2)
project(science_robot)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  duckietown_msgs
)

## Declare this project as a catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge duckietown_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

###########
## Install ##
###########

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/science_robot_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

