// Copyright 2020 Feiler
#include "OperatorManager.h"

OperatorManager::OperatorManager(const std::string& pathToInitialIpAddressFile,
        const std::string& searchedKey)
    : ip_addr_checker(new IPv4ValidityChecker), number_received_packages(0) {

    init_its_status_msg_data();

    // create window
    its_operatorManagerWidget = new OperatorManagerWidget(pathToInitialIpAddressFile,
        searchedKey, this);

    connect(its_operatorManagerWidget, &OperatorManagerWidget::signal_on_PushButton_Connect_clicked,
            this, &OperatorManager::handle_signal_on_connect_clicked);
    connect(its_operatorManagerWidget, &OperatorManagerWidget::signal_on_PushButton_Disconnect_clicked,
            this, &OperatorManager::handle_signal_on_disconnect_clicked);
    connect(its_operatorManagerWidget, &OperatorManagerWidget::signal_on_PushButton_Start_clicked,
            this, &OperatorManager::handle_signal_on_start_clicked);
    connect(its_operatorManagerWidget, &OperatorManagerWidget::signal_on_PushButton_Stop_clicked,
            this, &OperatorManager::handle_signal_on_stop_clicked);

    connect(this, &OperatorManager::signal_Connect,
            its_operatorManagerWidget, &OperatorManagerWidget::on_PushButton_Connect_clicked);
    connect(this, &OperatorManager::signal_Start,
            its_operatorManagerWidget, &OperatorManagerWidget::on_PushButton_Start_clicked);
    connect(this, &OperatorManager::signal_Stop,
            its_operatorManagerWidget, &OperatorManagerWidget::on_PushButton_Stop_clicked);
    connect(this, &OperatorManager::signal_Disconnect,
            its_operatorManagerWidget, &OperatorManagerWidget::on_PushButton_Disconnect_clicked);

    connect(this, &OperatorManager::signal_vehicle_emergency_stop_released,
            its_operatorManagerWidget, &OperatorManagerWidget::change_emergency_stop_released);
    connect(this, &OperatorManager::signal_vehicle_lat_approved,
            its_operatorManagerWidget, &OperatorManagerWidget::lat_approved);
    connect(this, &OperatorManager::signal_vehicle_long_approved,
            its_operatorManagerWidget, &OperatorManagerWidget::long_approved);

    connect(this, &OperatorManager::signal_update_nav_status,
            its_operatorManagerWidget, &OperatorManagerWidget::set_nav_status);
    connect(this, &OperatorManager::signal_update_gps_pos_type,
            its_operatorManagerWidget, &OperatorManagerWidget::set_gps_pos_type);

    connect(its_operatorManagerWidget, &OperatorManagerWidget::signal_control_mode_changed,
            this, &OperatorManager::set_control_mode_to_its_status_msg);
    connect(its_operatorManagerWidget, &OperatorManagerWidget::signal_input_device_changed,
            this, &OperatorManager::call_input_device_change_service);
    connect(its_operatorManagerWidget, &OperatorManagerWidget::signal_video_rate_control_mode_changed,
            this, &OperatorManager::set_video_rate_control_mode_to_its_status_msg);
    connect(this, &OperatorManager::new_vehicle_timestamp,
            its_operatorManagerWidget, &OperatorManagerWidget::update_vehicle_tx_label);
    connect(this, &OperatorManager::new_operator_timestamp,
            its_operatorManagerWidget, &OperatorManagerWidget::update_operator_tx_label);

    check_timer = new QTimer(this);
    connect(check_timer, &QTimer::timeout,
            this, &OperatorManager::update_gui_if_different_from_its_tod_status);
    connect(check_timer, &QTimer::timeout,
            this, &OperatorManager::close_application_if_needed);
    connect(check_timer, &QTimer::timeout, this, [this]{
            tod_msgs::Status tmpStudyLeaderWishes;
            if ( its_ros_loop.gotNewStudyLeaderWishes(tmpStudyLeaderWishes) ) {
                processStudyLeaderWishes(tmpStudyLeaderWishes);
            }
        });
    check_timer->start(10);
}

OperatorManager::~OperatorManager() {
    if ( its_operatorManagerWidget != nullptr ) { delete its_operatorManagerWidget; }
    check_timer->stop();
    delete check_timer;
}

void OperatorManager::handle_signal_on_connect_clicked(
        const std::string &ip_addr_broker,
        const std::string &ip_addr_operator) {
    // establish connection
    bool successful_mqtt_client_initialization =
            create_mqtt_client("op_client1", ip_addr_broker);
    if (successful_mqtt_client_initialization) {
        set_mqtt_pub_topic_to("Operator/Manager/status_msg");
        set_mqtt_callback_to_topic("Vehicle/Manager/status_msg");
        start_frequent_mqtt_pub(100);
        set_ip_addr_of_its_status_msg(ip_addr_broker, ip_addr_operator);
        set_tod_status_of_its_status_msg(tod_msgs::Status::TOD_STATUS_UPLINK_ONLY);
    }
}

void OperatorManager::handle_signal_on_disconnect_clicked() {
    set_tod_status_of_its_status_msg(tod_msgs::Status::TOD_STATUS_IDLE);
    if ( mqtt_pub_loop_is_running ) {
        stop_frequent_mqtt_pub();
        // tell the vehicle about the disconnection:
        pub_its_status_msg_to_broker();
    }
}

void OperatorManager::handle_signal_on_start_clicked() {
    set_tod_status_of_its_status_msg(tod_msgs::Status::TOD_STATUS_TELEOPERATION);
}

void OperatorManager::handle_signal_on_stop_clicked() {
    set_tod_status_of_its_status_msg(tod_msgs::Status::TOD_STATUS_UPLINK_ONLY);
}

void OperatorManager::set_control_mode_to_its_status_msg(const uint8_t control_mode) {
    its_mutex.lock();
    its_status_msg.operator_control_mode = control_mode;
    its_mutex.unlock();
    update_operator_time_stamp();
}

void OperatorManager::call_input_device_change_service(const std::string& input_device) {
    tod_msgs::InputDevice input_device_request;
    input_device_request.request.input_device_directory = input_device;
    bool success =  ros::service::call("change_input_device",
        input_device_request);
    if ( !success ) {
        ROS_ERROR("Could not reach service server for input device change at %s",
            ros::this_node::getName().c_str());
    }
}

void OperatorManager::set_video_rate_control_mode_to_its_status_msg(const uint8_t video_rate_control_mode) {
    its_mutex.lock();
    its_status_msg.operator_video_rate_mode = video_rate_control_mode;
    its_mutex.unlock();
    update_operator_time_stamp();
}

bool OperatorManager::check_ip_addr_validity(const std::string& ip_addr) {
    return ip_addr_checker->validate(ip_addr);
}

bool OperatorManager::create_mqtt_client(const std::string& client_id,
    const std::string& ip_addr_broker) {
    if ( !check_ip_addr_validity(ip_addr_broker) ) {
        std::cerr << "In OperatorManager.cpp (102): Broker IP address format is not valid\n";
        return false;
    }
    its_client.reset(new tod_network::MqttClientTemplated<OperatorManager>
        (ip_addr_broker, client_id));
    if (its_client->is_connected()) {
        return true;
    } else {
        std::cerr << "In OperatorManager.cpp (110): Could not reach provided broker\n";
        return false;
    }
}

void OperatorManager::create_and_run_ros_thread() {
    its_thread = std::thread(&rosloop_operator::ros_run_spin_loop, &its_ros_loop,
        "OperatorManagerNode", "status_msg", &its_status_msg, &ros_terminated);
}

void OperatorManager::create_close_qt_thread() {
    its_close_qt_thread = std::thread(&OperatorManager::wait_to_shut_down_qt_loop, this);
}

void OperatorManager::wait_to_shut_down_qt_loop() {
    while ( ros::ok() ) {
    }
    wait_for_ros_thread_to_join();
    its_operatorManagerWidget->quitAll();
}

void OperatorManager::shutdown_qt_widget() {
    its_operatorManagerWidget->quitAll();
}

void OperatorManager::wait_for_ros_thread_to_join() {
    its_thread.join();
    printf("ros thread joined\n");
}

void OperatorManager::set_mqtt_pub_topic_to(const std::string& topic) {
    its_mqtt_status_msg_topic = topic;
}

void OperatorManager::pub_its_status_msg_to_broker() {
    update_operator_time_stamp();
    its_mutex.lock();
    ros::SerializedMessage rosSer = ros::serialization::serializeMessage
        <tod_msgs::Status>(its_status_msg);
    its_mutex.unlock();
    its_client->publish(its_mqtt_status_msg_topic, 1,
        (char*) rosSer.message_start, rosSer.num_bytes);
}

void OperatorManager::set_mqtt_callback_to_topic(const std::string& topic_name) {
    its_client->subscribe(topic_name, 1,
        &OperatorManager::callback_await_response, this);
}

void OperatorManager::callback_await_response(const mqtt::const_message_ptr msg) {
    ros::serialization::IStream stream((uint8_t*) msg->get_payload_str().c_str(),
        msg->get_payload().length());
    tod_msgs::Status tmp;
    ros::serialization::Serializer<tod_msgs::Status>::read(stream, tmp);
    set_vehicle_part_of_its_status_msg(tmp);
    emit new_vehicle_timestamp((unsigned int) its_status_msg.vehicle_header.stamp.sec);
    if ( number_received_packages >= INT_MAX ) {
        number_received_packages = 0;
    }
    ++number_received_packages;
}

void OperatorManager::set_tod_status_of_its_status_msg(const uint8_t connection_status) {
    its_mutex.lock();
    its_status_msg.tod_status = connection_status;
    its_mutex.unlock();
    update_operator_time_stamp();
}

void OperatorManager::init_its_status_msg_data() {
    its_status_msg.tod_status = tod_msgs::Status::TOD_STATUS_IDLE;
    its_status_msg.vehicle_lat_approved = false;
    its_status_msg.vehicle_long_approved = false;
    its_status_msg.vehicle_emergency_stop_released = false;
    its_status_msg.vehicle_vehicle_ip_address = "127.0.0.1";
    its_status_msg.operator_control_mode =
        tod_msgs::Status::CONTROL_MODE_DIRECT;
    its_status_msg.operator_video_rate_mode =
        tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE;
    its_status_msg.operator_input_device =
        InputDevice::FANATEC;
    update_operator_time_stamp();
}

void OperatorManager::set_operator_part_of_its_status_msg(const tod_msgs::Status& msg) {
    its_mutex.lock();
    its_status_msg.operator_header = msg.operator_header;
    its_status_msg.operator_control_mode = msg.operator_control_mode;
    its_status_msg.operator_video_rate_mode = msg.operator_video_rate_mode;
    its_status_msg.operator_broker_ip_address = msg.operator_broker_ip_address;
    its_status_msg.operator_operator_ip_address = msg.operator_operator_ip_address;
    its_mutex.unlock();
    update_operator_time_stamp();
}

void OperatorManager::set_vehicle_part_of_its_status_msg(const tod_msgs::Status& msg) {
    its_mutex.lock();
    its_status_msg.vehicle_header = msg.vehicle_header;
    its_status_msg.vehicle_control_mode = msg.vehicle_control_mode;
    its_status_msg.vehicle_lat_approved = msg.vehicle_lat_approved;
    its_status_msg.vehicle_long_approved = msg.vehicle_long_approved;
    its_status_msg.vehicle_vehicle_ip_address = msg.vehicle_vehicle_ip_address;
    its_status_msg.vehicle_emergency_stop_released = msg.vehicle_emergency_stop_released;
    its_status_msg.vehicle_nav_status = msg.vehicle_nav_status;
    its_status_msg.vehicle_gps_pos_type = msg.vehicle_gps_pos_type;
    its_mutex.unlock();
}

void OperatorManager::set_ip_addr_of_its_status_msg(
    const std::string& ip_addr_broker, const std::string& ip_addr_operator) {
    its_mutex.lock();
    its_status_msg.operator_broker_ip_address = ip_addr_broker;
    its_status_msg.operator_operator_ip_address = ip_addr_operator;
    its_status_msg.vehicle_vehicle_ip_address = ip_addr_broker;
    its_mutex.unlock();
    update_operator_time_stamp();
}

void OperatorManager::start_frequent_mqtt_pub(const int frequency) {
    its_mqtt_thread = std::thread(&OperatorManager::mqtt_pub_loop, this,
                                  frequency);
}

void OperatorManager::mqtt_pub_loop(const int frequency) {
    mqtt_pub_loop_is_running = true;
    while ( !stop_mqtt_pub_loop ) {
        pub_its_status_msg_to_broker();
        emit new_operator_timestamp((unsigned int) ros::Time::now().sec);
        usleep((unsigned int) frequency*1000);
    }
    stop_mqtt_pub_loop = false;
    mqtt_pub_loop_is_running = false;
}

void OperatorManager::stop_frequent_mqtt_pub() {
    stop_mqtt_pub_loop = true;
    its_mqtt_thread.join();
    printf("mqtt pub thread joined\n");
}

int OperatorManager::num_rec_mqtt_packages() {
    return number_received_packages;
}

bool OperatorManager::check_if_mqtt_client_is_connected() {
    if ( its_client == nullptr ) { return false; }
    return its_client->is_connected();
}

tod_msgs::Status OperatorManager::get_its_status_msg() {
    return its_status_msg;
}

void OperatorManager::update_operator_time_stamp() {
    ros::Time::init();
    its_mutex.lock();
    its_status_msg.operator_header.stamp = ros::Time::now();
    its_mutex.unlock();
}

void OperatorManager::stop_mqtt_if_running() {
    if ( mqtt_pub_loop_is_running ) {
        stop_frequent_mqtt_pub();
    }
}

void OperatorManager::close_application_if_needed() {
    if ( ros_terminated ) {
        ros_terminated = false;
        printf("\n");
        printf("OPERATORMANAGER: STARTED TO SHUTDOWN THE APPLICATION - WAITING FOR THREADS TO JOIN\n");
        stop_mqtt_if_running();
        wait_for_ros_thread_to_join();
        QApplication::quit();
        printf("\n");
    }
}

void OperatorManager::stop_mqtt_if_idle() {
    if ( its_status_msg.tod_status == tod_msgs::Status::TOD_STATUS_IDLE ) {
        stop_mqtt_if_running();
    }
}

void OperatorManager::update_gui_if_different_from_its_tod_status() {
    tod_msgs::Status gui_status = its_operatorManagerWidget->get_gui_status();
    if (gui_status.vehicle_emergency_stop_released != its_status_msg.vehicle_emergency_stop_released) {
        emit signal_vehicle_emergency_stop_released(its_status_msg.vehicle_emergency_stop_released);
    }
    if ( gui_status.vehicle_lat_approved != its_status_msg.vehicle_lat_approved ) {
        emit signal_vehicle_lat_approved(its_status_msg.vehicle_lat_approved);
    }
    if ( gui_status.vehicle_long_approved != its_status_msg.vehicle_long_approved ) {
        emit signal_vehicle_long_approved(its_status_msg.vehicle_long_approved);
    }

    if ( gui_status.vehicle_nav_status != its_status_msg.vehicle_nav_status ) {
        emit signal_update_nav_status(its_status_msg.vehicle_nav_status);
    }
    if ( gui_status.vehicle_gps_pos_type != its_status_msg.vehicle_gps_pos_type ) {
        emit signal_update_gps_pos_type(its_status_msg.vehicle_gps_pos_type);
    }
}

void OperatorManager::processStudyLeaderWishes(const tod_msgs::Status studyLeaderWishes) {
    if ( its_status_msg.tod_status == tod_msgs::Status::TOD_STATUS_IDLE &&
    studyLeaderWishes.tod_status == tod_msgs::Status::TOD_STATUS_UPLINK_ONLY) {
        emit signal_Connect(studyLeaderWishes.operator_broker_ip_address,
            studyLeaderWishes.operator_operator_ip_address);
    }
    if ( its_status_msg.tod_status == tod_msgs::Status::TOD_STATUS_UPLINK_ONLY &&
    studyLeaderWishes.tod_status == tod_msgs::Status::TOD_STATUS_TELEOPERATION ) {
        set_control_mode_to_its_status_msg(studyLeaderWishes.operator_control_mode);
        emit signal_Start();
    }
    if ( its_status_msg.tod_status == tod_msgs::Status::TOD_STATUS_TELEOPERATION &&
    studyLeaderWishes.tod_status == tod_msgs::Status::TOD_STATUS_UPLINK_ONLY ) {
        emit signal_Stop();
    }
    if ( studyLeaderWishes.tod_status == tod_msgs::Status::TOD_STATUS_IDLE ) {
        emit signal_Disconnect();
    }
}
