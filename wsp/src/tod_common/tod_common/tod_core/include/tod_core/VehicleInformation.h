// Copyright 2021 Hoffmann
#pragma once
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <tod_helper/vehicle/Model.h>
#include "tod_core/BaseParameterHandler.h"
#include "tod_core/YamlLoader.h"

namespace tod_core {

class VehicleInformation : public BaseParameterHandler {
public:
    explicit VehicleInformation(ros::NodeHandle& nh);
    void load_parameters() override;

    std::string get_manufacturer() const;
    std::string get_type() const;
    float get_id() const;

private:
    std::string _manufacturer;
    std::string _type;
    int _id;
};
} // namespace tod_core
