// Copyright 2021 Schimpe
#include "tod_core/LidarParameters.h"

namespace tod_core {

LidarParameters::LidarParameters(ros::NodeHandle& nh) : BaseParameterHandler("Lidar") {
    load_parameters();
}

void LidarParameters::load_parameters() {
    update_vehicle_id();
    YamlLoader loader;
    if (!loader.load_from_path(get_path_to_config_files() + "/sensors-lidar.yaml")) {
        return;
    }

    _lidar_topics_namespace = loader.get_param<std::string>("lidar_topics_namespace");
    _laser_scan_name = loader.get_param<std::string>("laser_scan_name");
    _pointcloud_name = loader.get_param<std::string>("pointcloud_name");

    _sensors.clear();
    for (int i=0; i < 20; ++i) {
        std::string ns = std::string("lidar" + std::to_string(i));
        if (!loader.has_node(ns))
            continue;

        std::string name = loader.get_param<std::string>(ns, "name");
        LidarSensor& sensor = _sensors.emplace_back(name);

        sensor.is_3D = loader.get_param<bool>(ns, "is_3D");
        if (loader.has_node(ns, "construct_gridmap_from"))
            sensor.construct_gridmap_from = loader.get_param<bool>(ns, "construct_gridmap_from");
        if (loader.has_node(ns, "detect_on"))
            sensor.detect_on = loader.get_param<bool>(ns, "detect_on");
    }

    parameters_updated();
}

}; // namespace tod_core
