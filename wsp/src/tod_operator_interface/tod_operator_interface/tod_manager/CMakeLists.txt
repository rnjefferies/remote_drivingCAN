cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME tod_manager)
project(${PROJECT_NAME})

find_package(catkin REQUIRED COMPONENTS roscpp roslib tod_msgs tod_network tod_helper tod_core)
find_package(PahoMqttCpp REQUIRED)
find_package(Threads)

catkin_package(
    CATKIN_DEPENDS roscpp roslib tod_msgs tod_network tod_helper tod_core)

option(OPERATOR "Build operator targets" ON)
if (${OPERATOR})
    add_subdirectory(src/operator)
else()
    message("skipping ${PROJECT_NAME} operator targets")
endif()

option(VEHICLE "Build vehicle targets" ON)
if (${VEHICLE})
    add_subdirectory(src/vehicle)
else()
    message("skipping ${PROJECT_NAME} vehicle targets")
endif()

option(TEST "Build test targets" OFF)
if (${TEST})
    message("building ${PROJECT_NAME} test targets")
    add_subdirectory(test/)
endif()

install(DIRECTORY 
    launch
    yaml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


# node for ci - publishing connected on localhost at vehicle and operator side
add_executable(TestTodManager test/TestTodManagerNode.cpp)
add_dependencies(TestTodManager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(TestTodManager PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(TestTodManager ${catkin_LIBRARIES})
