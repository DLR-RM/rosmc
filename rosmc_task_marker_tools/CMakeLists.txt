cmake_minimum_required(VERSION 2.8.3)
project(rosmc_task_marker_tools)

find_package(catkin REQUIRED COMPONENTS
        rosmc_msgs
        geometry_msgs
        roscpp
        rviz
        tf
        )

find_package(OpenCV REQUIRED)

catkin_package(
        CATKIN_DEPENDS rosmc_msgs geometry_msgs roscpp rviz tf
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

link_directories(
        ${catkin_LIBRARY_DIRS}
)

set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    include(${QT_USE_FILE})
else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)
add_definitions("-Wall -Wunused -std=c++11")

set(SRC_FILES
        src/add_task_marker_2d_position_tool.cpp
        src/add_task_marker_2d_pose_tool.cpp
        src/add_task_marker_2d_area_tool.cpp
        src/quad_area.cpp
        )

add_library(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES
        plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)