cmake_minimum_required(VERSION 2.8.3)
project(rosmc_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    rosmc_interface_msgs
    std_msgs
    geometry_msgs
    visualization_msgs
)

add_message_files(
    FILES
    CurrentActions.msg
    MarkerStatus.msg
)

add_service_files(
    FILES
    AddAction.srv
    AddSync.srv
    AddTaskMarker.srv
    # AddTaskMarker2DArea.srv
    # AddTaskMarker2DPose.srv
    # AddTaskMarker2DPosition.srv
    AddTaskMarkerArea.srv
    DeleteAction.srv
    DeleteSync.srv
    EditAction.srv
    ExtendAddActionServices.srv
    GetIntMarker.srv
    GetIntMarkerVertices.srv
    LoadIntMarkers.srv
    LoadMission.srv
    NewMission.srv
    RenameMarker.srv
    ReorderActions.srv
    SampleRock.srv
    SaveIntMarkers.srv
    SaveMission.srv
    UpdateIntMarker.srv
    UpdateMarkerStatus.srv
)

generate_messages(
    DEPENDENCIES std_msgs geometry_msgs visualization_msgs rosmc_interface_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
    DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)

#install(PROGRAMS
#  scripts/advertise_trigger_mission_executor.py
#  DESTINATION
#  ${CATKIN_PACKAGE_BIN_DESTINATION}
#)