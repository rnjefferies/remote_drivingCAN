# ------------------------------------------------------------------------
# --------------------------- IPv4ValidityChecker ------------------------
# ------------------------------------------------------------------------
set(TARGET_CHECKER IPv4ValidityChecker)
set(CHECKER_SRC
    IPv4ValidityChecker_test.cpp
    )
add_executable(${TARGET_CHECKER} 
    ${CHECKER_SRC}
    ../src/operator/IPv4ValidityChecker.cpp
    )
target_include_directories(${TARGET_CHECKER} PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../src/operator)
add_dependencies(${TARGET_CHECKER} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${TARGET_CHECKER}
    PRIVATE
    ${catkin_LIBRARIES}
    PahoMqttCpp::paho-mqttpp3
    Threads::Threads
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    )
set_property(TARGET ${TARGET_CHECKER} PROPERTY CXX_STANDARD 17)

