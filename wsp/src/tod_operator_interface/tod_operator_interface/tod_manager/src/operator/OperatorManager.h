// Copyright 2020 Feiler
#pragma once
#include "tod_network/mqtt_client_templated.h"
#include "IpAddrValidityChecker.h"
#include "IPv4ValidityChecker.h"
#include <string>
#include <memory>
#include <mutex>
#include <deque>
#include "tod_msgs/Status.h"
#include "tod_msgs/inputDevice.h"
#include "tod_msgs/InputDevice.h"
#include "std_msgs/Time.h"
#include "rosthread.h"
#include "OperatorManagerWidget.h"
#include <QThread>
#include <QTimer>
#include <QApplication>

class OperatorManager  : public QObject  {
    Q_OBJECT

public:
    OperatorManager(const std::string& pathToInitialIpAddressFile,
        const std::string& searchedKey);
    ~OperatorManager();
    void create_and_run_ros_thread();
    void wait_for_ros_thread_to_join();
    bool create_mqtt_client(const std::string& client_id,
            const std::string& ip_addr_broker);
    void set_mqtt_callback_to_topic(const std::string& topic_name);
    void set_mqtt_pub_topic_to(const std::string& topic);
    void pub_its_status_msg_to_broker();
    void set_tod_status_of_its_status_msg(const uint8_t connection_status);
    void set_operator_part_of_its_status_msg(const tod_msgs::Status& msg);
    void set_vehicle_part_of_its_status_msg(const tod_msgs::Status& msg);

    bool check_ip_addr_validity(const std::string& ip_addr);

    void callback_await_response(const mqtt::const_message_ptr msg);

    void start_frequent_mqtt_pub(const int frequency);
    void stop_frequent_mqtt_pub();
    void wait_for_mqtt_thread_to_join();
    int num_rec_mqtt_packages();
    void wait_to_shut_down_qt_loop();
    void set_control_mode_to_its_status_msg(const uint8_t control_mode);
    void set_video_rate_control_mode_to_its_status_msg(const uint8_t video_rate_control_mode);
    void create_close_qt_thread();
    void call_input_device_change_service(const std::string& input_device);
    void shutdown_qt_widget();
    bool check_if_mqtt_client_is_connected();
    void handle_signal_on_connect_clicked(const std::string& ip_addr_broker,
                                          const std::string& ip_addr_operator);
    void handle_signal_on_start_clicked();
    void handle_signal_on_stop_clicked();
    void stop_mqtt_if_running();
    tod_msgs::Status get_its_status_msg();
    void stop_mqtt_if_idle();

    OperatorManagerWidget* its_operatorManagerWidget;

signals:
    void signal_Connect(const std::string& ip_addr_broker,
        const std::string& ip_addr_operator);
    void signal_Start();
    void signal_Stop();
    void signal_Disconnect();
    void new_vehicle_timestamp(unsigned int secondsOfTimestamp);
    void new_operator_timestamp(unsigned int secondsOfTimestamp);
    void signal_vehicle_emergency_stop_released(bool released);
    void signal_vehicle_lat_approved(bool approved);
    void signal_vehicle_long_approved(bool approved);
    void signal_update_nav_status(const std::string nav_status);
    void signal_update_gps_pos_type(const std::string gps_pos_type);

private:
    std::unique_ptr<tod_network::MqttClientTemplated<OperatorManager>> its_client;
    std::string its_mqtt_status_msg_topic;
    std::unique_ptr<IpAddrValidityChecker> ip_addr_checker;
    std::thread its_thread;
    rosloop_operator its_ros_loop;
    std::mutex its_mutex;
    std::thread its_mqtt_thread;
    int number_received_packages;
    std::thread its_close_qt_thread;
    bool stop_mqtt_pub_loop{ false };
    bool mqtt_pub_loop_is_running{ false };
    tod_msgs::Status its_status_msg;
    QTimer* check_timer;
    bool ros_terminated{ false };
    std::deque<bool> its_uptodate_storage;

    void init_its_status_msg_data();
    void mqtt_pub_loop(const int frequency);
    void set_ip_addr_of_its_status_msg(
            const std::string& ip_addr_broker, const std::string& ip_addr_operator);
    void handle_signal_on_disconnect_clicked();
    void update_operator_time_stamp();
    void close_application_if_needed();
    void update_gui_if_different_from_its_tod_status();
    void processStudyLeaderWishes(const tod_msgs::Status studyLeaderWishes);
};
