cmake_minimum_required(VERSION 2.8.3)
project(rosmc_task_marker_server)

find_package(catkin REQUIRED COMPONENTS
  rosmc_msgs
  interactive_markers
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS rosmc_msgs interactive_markers roscpp rospy std_msgs visualization_msgs
)

catkin_python_setup()

install(DIRECTORY launch meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/task_marker_2d_position_server_node.py
  scripts/task_marker_2d_pose_server_node.py
  scripts/task_marker_2d_area_server_node.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)
