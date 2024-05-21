find_package(GLM REQUIRED)

set(project_sources
    src/BaseFootprint.cpp
    src/Camera.cpp
    src/CoordinateSystem.cpp
    src/Display.cpp
    src/Floor.cpp
    src/Grid.cpp
    src/LaserScan.cpp
    src/Path.cpp
    src/TopView.cpp
    src/Video.cpp
    src/ObjectList.cpp
    src/Polygon.cpp
)

add_library(TodStandardEntities ${project_sources} include/TodStandardEntities/TodStandardEntities.h)
target_include_directories(TodStandardEntities
    PUBLIC
        include/
    PRIVATE
        ${catkin_INCLUDE_DIRS}
)
add_dependencies(TodStandardEntities ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(TodStandardEntities
    PUBLIC
    GLM::GLM
    tofGL
    ${catkin_LIBRARIES}
)
target_compile_features(TodStandardEntities PUBLIC cxx_std_17)

install(TARGETS TodStandardEntities
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
