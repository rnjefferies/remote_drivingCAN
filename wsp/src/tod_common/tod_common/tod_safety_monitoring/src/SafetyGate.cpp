// Copyright 2021 Simon Hoffmann
#include "tod_safety_monitoring/SafetyGate.h"

SafetyGate::SafetyGate(ros::NodeHandle &nh) : _nh(nh) {
    _trajSubs = _nh.subscribe("/trajectory_in", 5, &SafetyGate::callback_trajectory, this);
    _statusSubs = _nh.subscribe("/status_msg", 5, &SafetyGate::callback_status, this);
    _trajPub = _nh.advertise<tod_msgs::Trajectory>("/trajectory_out", 5);
    _gateStatePub = _nh.advertise<tod_safety_monitoring::GateState>("gate_state", 5);

    std::string ns{""}, configFile{""};
    _nh.getParam(ros::this_node::getName() + "/namespace_for_safety_issues", ns);
    _nh.getParam(ros::this_node::getName() + "/mandatoryTopics", configFile);

    bool verbose{true}, debug{false};
    _nh.getParam(ros::this_node::getName() + "/verbose", verbose);
    set_verbosity(verbose);
    _nh.getParam(ros::this_node::getName() + "/debug", debug);
    if (debug) // print ROS_DEBUG
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();

    _monitoringObjects = std::make_unique<tod_safety_monitoring::MonitoringObjectHandler>(_nh, configFile, ns);
}

void SafetyGate::run() {
    ros::Rate r(100);
    ros::Duration(1.5).sleep();

    while (ros::ok()) {
        ros::spinOnce();
        update();
        publish_gate_state();
        r.sleep();
    }
}

void SafetyGate::publish_gate_state() {
    tod_safety_monitoring::GateState state;
    state.header.stamp = ros::Time::now();
    state.state = static_cast<uint8_t>(get_current_state());
    _gateStatePub.publish(state);
}

void SafetyGate::check_and_clear_monitoring_objects() {
    _monitoringObjects->check_timeout();
    _monitoringObjects->update_error_lvl();
    switch (_monitoringObjects->get_current_error_lvl()) {
        case tod_safety_monitoring::ErrorLvl::OK:
            handle_gate_event(GateEvent::OPEN);
            break;
        case tod_safety_monitoring::ErrorLvl::WARNING:
            ROS_WARN_STREAM_THROTTLE(5.0, ros::this_node::getName() << "\nIn State: " << get_current_state() <<"\n"
                << _monitoringObjects->get_issues_as_string(tod_safety_monitoring::ErrorLvl::WARNING));
            handle_gate_event(GateEvent::WARN);
            break;
        case tod_safety_monitoring::ErrorLvl::ERROR:
            ROS_ERROR_STREAM_THROTTLE(5.0, ros::this_node::getName() << "\nIn State: " << get_current_state() <<"\n"
                << _monitoringObjects->get_issues_as_string(tod_safety_monitoring::ErrorLvl::ERROR));
            handle_gate_event(GateEvent::CLOSE);
            break;
    }
    _monitoringObjects->clear();
}

void SafetyGate::callback_trajectory(const tod_msgs::Trajectory& msg) {
    _firstTrajectory = true;
    _latestTrajectory = msg;
}

void SafetyGate::callback_status(const tod_msgs::Status& msg) {
    if (!safe_corridor_started(_currentTodState) && safe_corridor_started(msg))
        _operatorChangedToStart = true;
    if (!safe_corridor_started(msg))
            handle_gate_event(GateEvent::RESET);
    _currentTodState = msg;
}

void SafetyGate::startup_on_update() {
    _firstTrajectory = false;
    _operatorChangedToStart = false;
    _monitoringObjects->clear();
    if (_monitoringObjects->init_monitoring_objects())
        handle_gate_event(GateEvent::INITIALIZED);
}

void SafetyGate::initialized_on_update() {
    if (_firstTrajectory && _operatorChangedToStart) {
        check_and_clear_monitoring_objects();
        return;
    }
    if (!_firstTrajectory)
        ROS_DEBUG_STREAM_THROTTLE(5.0, ros::this_node::getName()
            << ": Waiting for first Trajectory!");
    if (!_operatorChangedToStart)
        ROS_DEBUG_STREAM_THROTTLE(5.0, ros::this_node::getName()
            << ": Waiting for Operator to Start SafeCorridor Session!");
    _monitoringObjects->clear();
}

void SafetyGate::open_on_update() {
    check_and_clear_monitoring_objects();
    _safeTrajectory = _latestTrajectory;
    _trajPub.publish(_safeTrajectory);
}

void SafetyGate::closed_on_update() {
    check_and_clear_monitoring_objects();
    _trajPub.publish(_safeTrajectory);
}

void SafetyGate::warn_on_update() {
    check_and_clear_monitoring_objects();
    _safeTrajectory = _latestTrajectory;
    _trajPub.publish(_safeTrajectory);
}

bool safe_corridor_started(const tod_msgs::Status& state) {
    return state.vehicle_control_mode == TOD_STATE::CONTROL_MODE_SAFECORRIDOR
            && state.tod_status == TOD_STATE::TOD_STATUS_TELEOPERATION;
}
