// Copyright 2021 Hoffmann
#pragma once
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <tod_helper/vehicle/Model.h>
#include "tod_core/BaseParameterHandler.h"
#include "tod_core/YamlLoader.h"
#include "geometry_msgs/TransformStamped.h"

namespace tod_core {

class TransformParameters : public BaseParameterHandler {
public:
    explicit TransformParameters(ros::NodeHandle& nh);
    void load_parameters() override;
    std::vector<geometry_msgs::TransformStamped> get_transforms();
    void updateStamp();

private:
    std::vector<geometry_msgs::TransformStamped> _transforms;
};

} // namespace tod_core
