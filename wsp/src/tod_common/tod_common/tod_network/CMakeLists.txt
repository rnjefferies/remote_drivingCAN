cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME tod_network)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(catkin REQUIRED COMPONENTS roscpp tod_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp tod_msgs
    )

add_subdirectory(src)
