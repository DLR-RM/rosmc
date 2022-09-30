cmake_minimum_required(VERSION 2.8.3)
project(rosmc_mission_server)

find_package(catkin REQUIRED COMPONENTS
  rosmc_msgs
  rosmc_interface_msgs
  interactive_markers
  roscpp
  roslib
  rospy
)

catkin_package(
  CATKIN_DEPENDS rosmc_msgs rosmc_interface_msgs interactive_markers roscpp roslib rospy
)

install(DIRECTORY configs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/mission_server.py
  src/agent_action_dict.py
  scripts/call_add_action_service.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)
