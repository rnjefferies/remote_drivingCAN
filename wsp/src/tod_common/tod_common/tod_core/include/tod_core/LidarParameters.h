// Copyright 2021 Schimpe
#pragma once
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include "tod_core/BaseParameterHandler.h"
#include "tod_core/YamlLoader.h"

namespace tod_core {

class LidarParameters :  public BaseParameterHandler {
public:
    struct LidarSensor {
        std::string name{""};
        bool is_3D{false};
        bool construct_gridmap_from{false};
        bool detect_on{false};
        explicit LidarSensor(const std::string& myName) : name{myName} { }
    };

    explicit LidarParameters(ros::NodeHandle& nh);
    void load_parameters() override;

    std::string get_lidar_topics_namespace() const { return _lidar_topics_namespace; }
    std::string get_laser_scan_name() const { return _laser_scan_name; }
    std::string get_pointcloud_name() const { return _pointcloud_name; }
    int get_number_of_sensors() const { return _sensors.size(); }
    const std::vector<LidarSensor>& get_sensors() const { return _sensors; }

private:
    std::string _lidar_topics_namespace{""};
    std::string _laser_scan_name{""};
    std::string _pointcloud_name{""};
    std::vector<LidarSensor> _sensors;
};

}; // namespace tod_core
