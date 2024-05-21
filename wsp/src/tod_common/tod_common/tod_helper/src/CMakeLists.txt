set(TARGETS ${TARGETS} TrajectoryViz)
add_executable(TrajectoryViz TrajectoryViz.cpp)
set_property(TARGET TrajectoryViz PROPERTY CXX_STANDARD 17)
target_link_libraries(TrajectoryViz ${catkin_LIBRARIES})
target_include_directories(TrajectoryViz PRIVATE ${catkin_INCLUDE_DIRS} ../include)

set(TARGETS ${TARGETS} ColoredPolygonViz)
add_executable(ColoredPolygonViz ColoredPolygonViz.cpp)
set_property(TARGET ColoredPolygonViz PROPERTY CXX_STANDARD 17)
target_link_libraries(ColoredPolygonViz ${catkin_LIBRARIES})
target_include_directories(ColoredPolygonViz PRIVATE ${catkin_INCLUDE_DIRS} ../include)

set(TARGETS ${TARGETS} ObjectListViz)
add_executable(ObjectListViz ObjectListViz.cpp)
set_property(TARGET ObjectListViz PROPERTY CXX_STANDARD 17)
target_link_libraries(ObjectListViz ${catkin_LIBRARIES})
target_include_directories(ObjectListViz PRIVATE ${catkin_INCLUDE_DIRS} ../include)

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
