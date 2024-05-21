cmake_minimum_required(VERSION 3.10)
find_package(Threads REQUIRED)
find_package(PahoMqttCpp REQUIRED)

add_library(tod_network SHARED
    udp_sender.cpp
    udp_receiver.cpp
    tod_sender.cpp
    tod_receiver.cpp
    mqtt_client.cpp
    mqtt_client_templated.cpp
)

target_include_directories(tod_network
    PUBLIC ../include ${catkin_INCLUDE_DIRS}
)

target_link_libraries(tod_network
    ${catkin_LIBRARIES}
    PahoMqttCpp::paho-mqttpp3
    Threads::Threads
)

install(DIRECTORY ../include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS tod_network
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
