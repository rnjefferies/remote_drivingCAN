cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME tod_transform)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(DEPRECATION_FLAG "-Wno-deprecated-declarations -Wno-format")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEPRECATION_FLAG}")
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
        roscpp nav_msgs tf2_ros tod_core)

catkin_package(
    CATKIN_DEPENDS roscpp nav_msgs tf2_ros
    )

set(TARGETS ${TARGETS} CommonOdomTransformPublisher)
add_executable(CommonOdomTransformPublisher src/CommonOdomTransformPublisherNode.cpp)
add_dependencies(CommonOdomTransformPublisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(CommonOdomTransformPublisher ${catkin_LIBRARIES})
target_include_directories(CommonOdomTransformPublisher PRIVATE ${catkin_INCLUDE_DIRS})

set(TARGETS ${TARGETS} CommonTransformTreePublisher)
add_executable(CommonTransformTreePublisher
    src/CommonTransformTreePublisherNode.cpp)

add_dependencies(CommonTransformTreePublisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(CommonTransformTreePublisher PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(CommonTransformTreePublisher ${catkin_LIBRARIES})

install(DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
