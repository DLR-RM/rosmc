cmake_minimum_required(VERSION 2.8.3)
project(rosmc_agents)

find_package(catkin REQUIRED COMPONENTS
  rosmc_msgs
  rosmc_interface_msgs
  geometry_msgs
  interactive_markers
  roscpp
  roslib
  rospy
  tf
  turtlesim
)

catkin_package(
  CATKIN_DEPENDS rosmc_msgs rosmc_interface_msgs geometry_msgs interactive_markers roscpp roslib rospy tf turtlesim
)

catkin_python_setup()

install(DIRECTORY launch meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions("-Wall -Wunused -std=c++11")

install(PROGRAMS
  src/turtle_tf_broadcaster.py
  src/turtles_int_marker_publisher.py
  src/turtles_status_publisher.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)
