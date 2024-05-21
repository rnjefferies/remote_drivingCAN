// Copyright 2021 Schimpe
#include "tod_core/CameraParameters.h"

namespace tod_core {

CameraParameters::CameraParameters(ros::NodeHandle& nh) : BaseParameterHandler("Camera") {
    load_parameters();
}

void CameraParameters::load_parameters() {
    update_vehicle_id();
    YamlLoader loader;
    if (!loader.load_from_path(get_path_to_config_files() + "/sensors-camera.yaml")) {
        return;
    }

    if (loader.has_node("camera_topics_namespace"))
        _camera_topics_namespace = loader.get_param<std::string>("camera_topics_namespace");

    if (loader.has_node("camera_image_name"))
        _camera_image_name = loader.get_param<std::string>("camera_image_name");

    _sensors.clear();
    for (int i=0; i < 20; ++i) {
        std::string ns = std::string("camera" + std::to_string(i));
        if (!loader.has_node(ns))
            continue;

        CameraSensor& sensor = _sensors.emplace_back();

        if (loader.has_node(ns, "vehicle_name"))
            sensor.vehicle_name = loader.get_param<std::string>(ns, "vehicle_name");
        if (loader.has_node(ns, "operator_name"))
            sensor.operator_name = loader.get_param<std::string>(ns, "operator_name");
        if (loader.has_node(ns, "host"))
            sensor.hostname = loader.get_param<std::string>(ns, "host");
        if (loader.has_node(ns, "is_fisheye"))
            sensor.is_fisheye = loader.get_param<bool>(ns, "is_fisheye");
        if (loader.has_node(ns, "ip_offset"))
            sensor.ip_offset = loader.get_param<int>(ns, "ip_offset");
        if (loader.has_node(ns, "stream_on_connect"))
            sensor.stream_on_connect = loader.get_param<bool>(ns, "stream_on_connect");
        if (loader.has_node(ns, "project_on"))
            sensor.project_on = loader.get_param<bool>(ns, "project_on");
        if (loader.has_node(ns, "is_jpeg"))
            sensor.is_jpeg = loader.get_param<bool>(ns, "is_jpeg");
        if (loader.has_node(ns, "scalings"))
            sensor.scalings = loader.get_param<std::vector<std::string>>(ns, "scalings");
        if (loader.has_node(ns, "transition_bitrates"))
            sensor.transition_bitrates = loader.get_param<std::vector<int>>(ns, "transition_bitrates");
        if (sensor.scalings.size() != sensor.transition_bitrates.size()) {
            ROS_WARN("%s: has different number of scalings (%ld) and transition bitrates (%ld)",
                sensor.vehicle_name.c_str(), sensor.scalings.size(), sensor.transition_bitrates.size());
        }

        if (sensor.vehicle_name.empty()) {
            if (loader.has_node(ns, "name"))
                sensor.vehicle_name = loader.get_param<std::string>(ns, "name");
        }

        if (sensor.operator_name.empty()) {
            if (loader.has_node(ns, "name"))
                sensor.operator_name = loader.get_param<std::string>(ns, "name");
        }
    }

    parameters_updated();
}

}; // namespace tod_core
