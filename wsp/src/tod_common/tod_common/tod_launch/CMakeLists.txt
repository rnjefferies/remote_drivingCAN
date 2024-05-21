cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME tod_launch)
project(${PROJECT_NAME})

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    INCLUDE_DIRS
    LIBRARIES tod_launch
    CATKIN_DEPENDS
    DEPENDS
)

install(DIRECTORY
    launch
    xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
