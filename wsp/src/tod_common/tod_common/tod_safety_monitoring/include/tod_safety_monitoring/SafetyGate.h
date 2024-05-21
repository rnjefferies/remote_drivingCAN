// Copyright 2021 Simon Hoffmann
#pragma once

#include "ros/ros.h"
#include "ros/debug.h"
#include <vector>
#include <map>
#include <thread>
#include <tod_safety_monitoring/SafetyState.h>
#include "std_msgs/String.h"
#include <tod_safety_monitoring/IssueBuffer.h>
#include "sensor_msgs/Image.h"
#include "tod_msgs/Trajectory.h"
#include "tod_msgs/Status.h"
#include "tod_safety_monitoring/MonitoringObjectHandler.h"
#include "tod_safety_monitoring/BaseGate.h"
#include "tod_safety_monitoring/GateState.h"

using TOD_STATE = tod_msgs::Status;
using GateEvent = tod_safety_monitoring::Event;
class SafetyGate: public tod_safety_monitoring::BaseGate {
    public:
        explicit SafetyGate(ros::NodeHandle &nh);
        void check_and_clear_monitoring_objects();
        void run();
        void open_on_update() override;
        void startup_on_update() override;
        void initialized_on_update() override;
        void closed_on_update() override;
        void warn_on_update() override;

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _trajSubs, _statusSubs;
        ros::Publisher _trajPub, _gateStatePub;
        tod_msgs::Trajectory _safeTrajectory;
        tod_msgs::Trajectory _latestTrajectory;
        bool _firstTrajectory{false};
        bool _operatorChangedToStart{false};
        tod_msgs::Status _currentTodState;
        std::shared_ptr<tod_safety_monitoring::MonitoringObjectHandler> _monitoringObjects;
        void callback_trajectory(const tod_msgs::Trajectory& msg);
        void callback_status(const tod_msgs::Status& msg);
        void publish_gate_state();
};

bool safe_corridor_started(const tod_msgs::Status& state);
