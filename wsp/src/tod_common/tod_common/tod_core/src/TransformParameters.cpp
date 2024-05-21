// Copyright 2021 Hoffmann
#include "tod_core/TransformParameters.h"

namespace tod_core {

TransformParameters::TransformParameters(ros::NodeHandle& nh) : BaseParameterHandler("Transform") {
    load_parameters(); // get first Parameter Set
}

void TransformParameters::load_parameters() {
    update_vehicle_id();
    YamlLoader loader;
    if (!loader.load_from_path(get_path_to_config_files() + "/vehicle-transforms.yaml")) {
        return;
    }

    _transforms.clear();
    int i = 1;
    while (loader.has_node(std::string("Transform" + std::to_string(i)))) {
        std::string ns = std::string("Transform" + std::to_string(i));
        geometry_msgs::TransformStamped transformStamped;
        //stamp and frame_id
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = loader.get_param<std::string>(ns, "from");
        transformStamped.child_frame_id = loader.get_param<std::string>(ns, "to");
        // translation
        std::vector<double> translation_x_y_z;
        translation_x_y_z = loader.get_param<std::vector<double>>(ns, "translation_x_y_z");
        transformStamped.transform.translation.x = translation_x_y_z.at(0);
        transformStamped.transform.translation.y = translation_x_y_z.at(1);
        transformStamped.transform.translation.z = translation_x_y_z.at(2);
        //rotation
        std::vector<double> rot_ypr;
        rot_ypr = loader.get_param<std::vector<double>>(ns, "rotation_yaw_pitch_roll");
        tf2::Quaternion quat;
        quat.setRPY(rot_ypr.at(2), rot_ypr.at(1), rot_ypr.at(0));
        transformStamped.transform.rotation.x = quat.x();
        transformStamped.transform.rotation.y = quat.y();
        transformStamped.transform.rotation.z = quat.z();
        transformStamped.transform.rotation.w = quat.w();
        // pushback
        _transforms.push_back(transformStamped);
        i++;
    }

    parameters_updated();
}

std::vector<geometry_msgs::TransformStamped> TransformParameters::get_transforms() {
    return _transforms;
}

void TransformParameters::updateStamp() {
    for (auto &transform : _transforms) {
        transform.header.stamp = ros::Time::now();
    }
}
}; // namespace tod_core
