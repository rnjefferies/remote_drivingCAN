cmake_minimum_required(VERSION 2.8.3)
project(tod_pix_bridge) # Adjusted to the correct project name

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++17)

# Adjust the find_package call based on actual dependencies of tod_pix_bridge
# For example, if tod_pix_bridge doesn't use ackermann_msgs but uses can_msgs
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    nav_msgs
    can_msgs 
    tod_msgs
)

# Reflect any dependency changes in catkin_package as well
catkin_package(
    # INCLUDE_DIRS include # Uncomment and adjust if you have an 'include' directory
    CATKIN_DEPENDS roscpp sensor_msgs nav_msgs can_msgs tod_msgs
)

# Include directories
include_directories(
    ${catkin_INCLUDE_DIRS}
    # include # Uncomment if you have an 'include' directory
)

# Conditional compilation logic remains the same
option(VEHICLE "Build vehicle targets" ON)
if (${VEHICLE})
    add_subdirectory(src)
else()
    message("skipping ${PROJECT_NAME} vehicle targets")
endif()

# Installation of launch files remains unchanged
install(DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
