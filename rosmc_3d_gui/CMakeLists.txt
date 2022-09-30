cmake_minimum_required(VERSION 2.8.3)
project(rosmc_3d_gui)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
    CATKIN_DEPENDS
    DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)

# Install config files so that it can be accessed by `cissy run` and `conan install`
install(DIRECTORY configs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)