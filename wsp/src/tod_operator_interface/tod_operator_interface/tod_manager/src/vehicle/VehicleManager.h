// Copyright 2020 Feiler
#pragma once
#include <memory>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <thread>
#include "rosthread.h"
#include "tod_network/mqtt_client_templated.h"
#include "tod_msgs/Status.h"
#include "tod_msgs/SafetyDriverStatus.h"
#include <deque>

class VehicleManager {
public:
    VehicleManager();
    void create_and_run_ros_thread();
    void wait_for_ros_thread_to_join();
    bool create_mqtt_client_at_localhost(const std::string& client_id);
    void set_mqtt_callback_to_topic(const std::string& mqtt_topic_name);
    void set_mqtt_pub_topic_to(const std::string& topic);
    void pub_its_status_msg_to_broker();
    void set_tod_status_of_its_status_msg(const tod_msgs::Status& msg);
    void set_rosthread_msg_operator_part(const tod_msgs::Status& msg);
    void set_safety_driver_part_of_its_status_msg(const tod_msgs::Status& msg);
    void callback_tod_network_client(mqtt::const_message_ptr msg);
    void callback_safety_driver_status(const tod_msgs::SafetyDriverStatus&
        safety_driver_status_msg);
    void callback_nav_status(const std_msgs::String& nav_status);
    void callback_pos_type(const std_msgs::String& pos_type);
    int num_rec_mqtt_packages();
    tod_msgs::Status get_its_status_msg();

private:
    void init_rosthread_msg_data(); // man
    void change_tod_status_msg_if_disconnected(bool disconnection_detected);
    void update_vehicle_time_stamp();

    std::thread its_thread;
    rosloop_vehicle its_ros_loop;
    std::unique_ptr<tod_network::MqttClientTemplated<VehicleManager>> its_client;
    std::string its_mqtt_response_topic;
    std::mutex its_mutex;
    int number_received_packages;
    tod_msgs::Status its_status_msg;
};
