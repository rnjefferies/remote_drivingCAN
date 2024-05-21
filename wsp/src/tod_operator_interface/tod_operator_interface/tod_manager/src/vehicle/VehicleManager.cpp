// Copyright 2020 Feiler

#include "VehicleManager.h"

VehicleManager::VehicleManager() {
    init_rosthread_msg_data();
}

void VehicleManager::create_and_run_ros_thread() {
    its_thread = std::thread(&rosloop_vehicle::ros_run_spin_loop,
                             &its_ros_loop, "VehicleManagerNode", "status_msg",
                             &its_status_msg);
}

void VehicleManager::wait_for_ros_thread_to_join() {
    its_thread.join();
}

bool VehicleManager::create_mqtt_client_at_localhost(const std::string& client_id) {
    its_client.reset(new tod_network::MqttClientTemplated<VehicleManager>
        ("127.0.0.1", client_id));
    return its_client->is_connected();
}

void VehicleManager::set_mqtt_callback_to_topic(const std::string& mqtt_topic_name) {
    its_client->subscribe(mqtt_topic_name, 1,
        &VehicleManager::callback_tod_network_client,
        this);
}

void VehicleManager::callback_tod_network_client(mqtt::const_message_ptr msg) {
    ros::serialization::IStream stream((uint8_t*) msg->get_payload_str().c_str(),
        msg->get_payload().length());
    tod_msgs::Status tmp;
    ros::serialization::Serializer<tod_msgs::Status>::read(stream, tmp);
    set_tod_status_of_its_status_msg(tmp);
    set_rosthread_msg_operator_part(tmp);
    its_mutex.lock();
    its_status_msg.vehicle_vehicle_ip_address = tmp.operator_broker_ip_address;
    its_mutex.unlock();

    pub_its_status_msg_to_broker();
    if ( number_received_packages >= INT_MAX ) {
        number_received_packages = 0;
    }
    ++number_received_packages;
 }

void VehicleManager::callback_safety_driver_status(const tod_msgs::SafetyDriverStatus&
        safety_driver_status_msg) {
    tod_msgs::Status temporary_status_msg;
    temporary_status_msg.vehicle_emergency_stop_released =
        safety_driver_status_msg.vehicle_emergency_stop_released;
    temporary_status_msg.vehicle_lat_approved =
        safety_driver_status_msg.vehicle_lat_approved;
    temporary_status_msg.vehicle_long_approved =
        safety_driver_status_msg.vehicle_long_approved;
    set_safety_driver_part_of_its_status_msg(temporary_status_msg);
}


void VehicleManager::set_mqtt_pub_topic_to(const std::string& topic) {
    its_mqtt_response_topic = topic;
}

void VehicleManager::pub_its_status_msg_to_broker() {
    its_mutex.lock();
    ros::SerializedMessage rosSer = ros::serialization::serializeMessage
        <tod_msgs::Status>(its_status_msg);
    its_client->publish(its_mqtt_response_topic, 1, (char*) rosSer.message_start,
        rosSer.num_bytes);
    its_mutex.unlock();
}

void VehicleManager::init_rosthread_msg_data() {
    its_mutex.lock();
    its_status_msg.tod_status = tod_msgs::Status::TOD_STATUS_IDLE;
    its_status_msg.vehicle_lat_approved = false;
    its_status_msg.vehicle_long_approved = false;
    its_status_msg.vehicle_emergency_stop_released = false;
    its_status_msg.vehicle_vehicle_ip_address = "127.0.0.1";
    its_status_msg.vehicle_control_mode =
        tod_msgs::Status::CONTROL_MODE_DIRECT;
    its_status_msg.operator_video_rate_mode =
        tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}

void VehicleManager::set_tod_status_of_its_status_msg(const tod_msgs::Status& msg) {
    its_mutex.lock();
    its_status_msg.tod_status = msg.tod_status;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}

void VehicleManager::set_rosthread_msg_operator_part(const tod_msgs::Status& msg) {
    its_mutex.lock();
    its_status_msg.operator_header = msg.operator_header;
    its_status_msg.operator_control_mode = msg.operator_control_mode;
    its_status_msg.operator_video_rate_mode = msg.operator_video_rate_mode;
    its_status_msg.operator_input_device = msg.operator_input_device;
    its_status_msg.operator_broker_ip_address = msg.operator_broker_ip_address;
    its_status_msg.operator_operator_ip_address = msg.operator_operator_ip_address;
    its_status_msg.vehicle_control_mode = msg.operator_control_mode;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}

void VehicleManager::set_safety_driver_part_of_its_status_msg(const tod_msgs::Status& msg) {
    its_mutex.lock();
    its_status_msg.vehicle_lat_approved = msg.vehicle_lat_approved;
    its_status_msg.vehicle_long_approved = msg.vehicle_long_approved;
    its_status_msg.vehicle_emergency_stop_released = msg.vehicle_emergency_stop_released;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}

int VehicleManager::num_rec_mqtt_packages() {
    return number_received_packages;
}

void VehicleManager::change_tod_status_msg_if_disconnected(bool disconnection_detected) {
    if (disconnection_detected) {
        its_mutex.lock();
        its_status_msg.tod_status = tod_msgs::Status::TOD_STATUS_IDLE;
        its_mutex.unlock();
        update_vehicle_time_stamp();
    }
}

tod_msgs::Status VehicleManager::get_its_status_msg() {
    return its_status_msg;
}

void VehicleManager::update_vehicle_time_stamp() {
    ros::Time::init();
    its_mutex.lock();
    its_status_msg.vehicle_header.stamp = ros::Time::now();
    its_mutex.unlock();
}

void VehicleManager::callback_nav_status(const std_msgs::String& nav_status ) {
    const std::lock_guard<std::mutex> lock(its_mutex);
    its_status_msg.vehicle_nav_status = nav_status.data;
}

void VehicleManager::callback_pos_type(const std_msgs::String& pos_type) {
    const std::lock_guard<std::mutex> lock(its_mutex);
    its_status_msg.vehicle_gps_pos_type = pos_type.data;
}
