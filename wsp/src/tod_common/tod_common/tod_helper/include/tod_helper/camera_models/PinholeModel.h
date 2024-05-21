// Copyright 2021 Schimpe
#pragma once
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <vector>

struct PinholeModel {
public:
    int width_raw{0}, height_raw{0};
    float center_x{0.0f}, center_y{0.0f};
    float focal_x{0.0f}, focal_y{0.0f};

private:
    template<typename T>
    T get_param(const std::string &name, ros::NodeHandle &nh) {
        T var;
        if (!nh.getParam(name, var))
            ROS_ERROR("could not get param %s", name.c_str());
        return var;
    }

public:
    PinholeModel(const std::string &cameraName, const std::string &vehicleID) {
        const std::string nodeName{ros::this_node::getName()};
        const std::string camNs{nodeName + "/" + cameraName};
        // get path to yaml with params and load
        std::string pathToYaml = ros::package::getPath("tod_vehicle_config");
        pathToYaml += "/vehicle_config/" + vehicleID + "/camera-calibration/" + cameraName + ".yaml";
        std::string loadCmd{"rosparam load " + pathToYaml + " " + camNs};
        system(loadCmd.c_str());
        // get param values
        ros::NodeHandle nh;
        width_raw = get_param<int>(camNs + "/image_width", nh);
        height_raw = get_param<int>(camNs + "/image_height", nh);
        auto calibData = get_param<std::vector<float>>(camNs + "/camera_matrix/data", nh);
        focal_x = calibData.at(0);
        focal_y = calibData.at(4);
        center_x = calibData.at(2);
        center_y = calibData.at(5);
    }

    bool point_on_image(const geometry_msgs::Point &pt, int &x_px, int &y_px,
                       const double scaling_x = 1.0, const double scaling_y = 1.0) const {
        x_px = int(scaling_x * focal_x * pt.x / pt.z + scaling_x * center_x);
        y_px = int(scaling_y * focal_y * pt.y / pt.z + scaling_y * center_y);
        return (1 <= x_px && x_px <= scaling_x * width_raw &&
                1 <= y_px && y_px <= scaling_y * height_raw);
    }
};
