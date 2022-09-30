cmake_minimum_required(VERSION 2.8.3)
project(rosmc_command_gui)

find_package(catkin REQUIRED COMPONENTS
  rosmc_msgs
  rosmc_interface_msgs
  geometry_msgs
  interactive_markers
  rospy
  rqt_gui
  rqt_gui_py
  tf
)


catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rosmc_msgs rosmc_interface_msgs geometry_msgs interactive_markers rospy rqt_gui rqt_gui_py tf
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS scripts/rosmc_command_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  resource BreezeStyleSheets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)