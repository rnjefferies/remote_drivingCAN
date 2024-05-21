// Copyright 2020 TUMFTM
#pragma once
#include <string>

namespace tod_network {

static const int RX_MQTT = 1883; // same for operator and vehicle
static const std::string AutoboxIp = "192.168.140.6";

enum VehiclePorts { // only what is sent via udp
    RX_PRIMARYCONTROL_COMMAND = 70000,
    RX_SAFECORRIDORCONTROL_COMMAND = 70001,
    RX_SECONDARY_COMMAND = 70003,
    RX_SAFETY_DRIVER_STATUS_AUTOBOX = 60000,
    RX_VEHICLEDATA_AUTOBOX = 60001,
    RX_VIDEO_RTSP = 8554,
};

enum AutoboxPorts {
    RX_DIRECTCONTROL_AUTOBOX_COMMAND = 30000
};

enum OperatorPorts { // only what is sent via udp
    RX_LIDAR_OBJECTLIST = 50000,
    RX_VEHICLESTATE_VEHICLEDATA = 50002,
    // 50003
    RX_VEHICLESTATE_ODOMETRY = 50005,
    RX_VEHICLESTATE_GPS = 50006,
    RX_SHAREDCONTROL_POLYGON = 50007,
    RX_SHAREDCONTROL_COMMAND = 50008,
    RX_SHAREDCONTROL_MPC_LOG = 50009,
    RX_SHAREDCONTROL_OBJECTS = 50010,
    RX_BITRATE_PREDICTIONS = 50011,
    RX_SHAREDCONTROL_SHADOW_COMMAND = 50012,
    RX_SHAREDCONTROL_SVC_LOG = 50013,
    RX_GATE_STATE = 50040,
    RX_LIDAR_DATA_RANGE_FROM = 50100,
    RX_LIDAR_DATA_RANGE_TO = 50199,
    RX_LIDAR_OBJECTS_RANGE_FROM = 50200,
    RX_LIDAR_OBJECTS_RANGE_TO = 50299,
    RX_LIDAR_OBJECT_MARKER_RANGE_FROM = 50300,
    RX_LIDAR_OBJECT_MARKER_RANGE_TO = 50399,
};

namespace MqttTopics {
static const std::string DesiredVideoConfig{"/Operator/Video/DesiredVideoConfig"};
static const std::string ActualVideoConfig{"/Vehicle/Video/ActualVideoConfig"};
static const std::string DesiredBitrateConfig{"/Operator/Video/DesiredBitrateConfig"};
static const std::string ActualBitrateConfig{"/Vehicle/Video/ActualBitrateConfig"};
};

}; // namespace tod_network
