cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME tod_input_devices)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS roscpp roslib tod_msgs sensor_msgs)

catkin_package(
        CATKIN_DEPENDS roscpp roslib tod_msgs sensor_msgs
)

option(OPERATOR "Build operator targets" ON)
if (${OPERATOR})
    add_subdirectory(src)
else()
    message("skipping ${PROJECT_NAME} operator targets")
endif()

install(DIRECTORY 
    launch
    config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
