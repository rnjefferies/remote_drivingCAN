cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME tod_safety_monitoring)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tod_msgs
    std_msgs
    message_generation
    tod_helper
    )
find_package(rostest REQUIRED)

add_message_files(
    DIRECTORY msg
    FILES SafetyState.msg SafetyIssue.msg GateState.msg
    )

generate_messages(DEPENDENCIES std_msgs tod_safety_monitoring)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp message_generation tod_msgs
    DEPENDS
    )

set(TARGETS ${TARGETS} ${PROJECT_NAME})
add_library(${PROJECT_NAME}
    src/SafetyMonitor.cpp
    src/IssueBuffer.cpp
    src/ErrorLvl.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${catkin_INCLUDE_DIRS} src PUBLIC include)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES})

set(TARGETS ${TARGETS} safety_gate)
add_executable(safety_gate
    src/SafetyGateNode.cpp
    src/SafetyGate.cpp
    src/GateStateMachine.cpp
    src/MonitoringObjectHandler.cpp)
target_include_directories(safety_gate PRIVATE ${catkin_INCLUDE_DIRS} src PUBLIC include)
add_dependencies(safety_gate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(safety_gate PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_features(safety_gate PUBLIC cxx_std_17)

set(TARGETS ${TARGETS} VehicleWatchdog)
add_executable(VehicleWatchdog src/VehicleWatchdog.cpp)
target_include_directories(VehicleWatchdog PRIVATE ${catkin_INCLUDE_DIRS} src PUBLIC include)
add_dependencies(VehicleWatchdog ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleWatchdog PRIVATE ${catkin_LIBRARIES} ${PROJECT_NAME})
target_compile_features(VehicleWatchdog PUBLIC cxx_std_17)

set(TARGETS ${TARGETS} OperatorWatchdog)
add_executable(OperatorWatchdog src/OperatorWatchdog.cpp)
target_include_directories(OperatorWatchdog PRIVATE ${catkin_INCLUDE_DIRS} src PUBLIC include)
add_dependencies(OperatorWatchdog ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorWatchdog PRIVATE ${catkin_LIBRARIES} ${PROJECT_NAME})
target_compile_features(OperatorWatchdog PUBLIC cxx_std_17)

install(DIRECTORY 
    config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} ${TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
