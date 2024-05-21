add_executable(ControlCmdToCan ControlCmdToCanNode.cpp)
add_dependencies(ControlCmdToCan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ControlCmdToCan ${catkin_LIBRARIES})

add_executable(VehicleDataFromCan VehicleDataFromCanNode.cpp)
add_dependencies(VehicleDataFromCan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleDataFromCan ${catkin_LIBRARIES})

install(TARGETS
    ControlCmdToCan
    VehicleDataFromCan
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
