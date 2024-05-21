// Copyright 2020 Feiler

#include "ros/ros.h"
#include "VehicleManager.h"

#include <iostream>
#include <fstream>
#include <string>
#include <ostream>
#include <streambuf>

void init_vehicle_manager(VehicleManager* veh_man) {
    veh_man->create_and_run_ros_thread();
    veh_man->create_mqtt_client_at_localhost("/Vehicle/Manager");
    veh_man->set_mqtt_callback_to_topic("Operator/Manager/status_msg");
    veh_man->set_mqtt_pub_topic_to("Vehicle/Manager/status_msg");
}

void wait_until_ros_is_shutdown(VehicleManager* veh_man) {
    veh_man->wait_for_ros_thread_to_join();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleManager");
    ros::NodeHandle nh;
    VehicleManager vehicleManager;
    init_vehicle_manager(&vehicleManager);

    ros::Subscriber safety_driver_status_subscriber =
        nh.subscribe("/Vehicle/VehicleBridge/safety_driver_status", 5, &VehicleManager::callback_safety_driver_status,
            &vehicleManager);
    ros::Subscriber nav_status_subscriber =
        nh.subscribe("/Vehicle/VehicleBridge/gps/nav_status", 5, &VehicleManager::callback_nav_status,
            &vehicleManager);
    ros::Subscriber pos_type_subscriber =
        nh.subscribe("/Vehicle/VehicleBridge/gps/pos_type", 5, &VehicleManager::callback_pos_type,
            &vehicleManager);

    wait_until_ros_is_shutdown(&vehicleManager);
    return 0;
}
