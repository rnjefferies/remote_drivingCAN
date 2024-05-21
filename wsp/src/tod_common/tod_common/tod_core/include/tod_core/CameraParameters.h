// Copyright 2021 Schimpe
#pragma once
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include "tod_core/BaseParameterHandler.h"
#include "tod_core/YamlLoader.h"

namespace tod_core {

class CameraParameters :  public BaseParameterHandler {
public:
    struct CameraSensor {
        std::string vehicle_name{""};
        std::string operator_name{""};
        bool is_fisheye{false};
        int ip_offset{0};
        bool project_on{false};
        bool stream_on_connect{true};
        bool is_jpeg{false};
        std::string hostname{""};
        std::vector<std::string> scalings{"1p000"};
        std::vector<int> transition_bitrates{10000};
        CameraSensor() = default;
    };

    explicit CameraParameters(ros::NodeHandle& nh);
    void load_parameters() override;

    std::string get_camera_topics_namespace() const { return _camera_topics_namespace; }
    std::string get_camera_image_name() const { return _camera_image_name; }
    const std::vector<CameraSensor>& get_sensors() const { return _sensors; }

private:
    std::string _camera_topics_namespace{"/Vehicle/Video"};
    std::string _camera_image_name{"/image_raw"};
    std::vector<CameraSensor> _sensors;
};

}; // namespace tod_core
