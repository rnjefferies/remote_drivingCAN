// Copyright 2021 Hoffmann
#include "tod_core/VehicleInformation.h"

namespace tod_core {

VehicleInformation::VehicleInformation(ros::NodeHandle& nh) : BaseParameterHandler("VehicleInfo") {
    load_parameters(); // get first Parameter Set
}

void VehicleInformation::load_parameters() {
    update_vehicle_id();
    YamlLoader loader;
    if (!loader.load_from_path(get_path_to_config_files() + "/vehicle-info.yaml")) {
        return;
    }
    _manufacturer = loader.get_param<std::string>("manufacturer");
    _type = loader.get_param<std::string>("type");
    _id = loader.get_param<int>("id");

    parameters_updated();
}

std::string VehicleInformation::get_manufacturer() const {
    return _manufacturer;
}

std::string VehicleInformation::get_type() const {
    return _type;
}

float VehicleInformation::get_id() const {
    return _id;
}
}; // namespace tod_core
