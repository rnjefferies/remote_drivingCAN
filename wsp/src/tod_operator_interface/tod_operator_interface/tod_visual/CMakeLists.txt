cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME tod_visual)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    tf2_ros
    roslib
    tod_msgs
    tod_helper
    tod_core
)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf2_ros roslib tod_msgs tod_helper
)

option(OPERATOR "Build operator targets" ON)
if (${OPERATOR})
    add_subdirectory(src)
else()
    message("skipping ${PROJECT_NAME} operator targets")
endif()

install(DIRECTORY 
    launch
    resources
    yaml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
