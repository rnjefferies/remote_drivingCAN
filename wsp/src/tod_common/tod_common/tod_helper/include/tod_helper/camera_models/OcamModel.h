// Copyright 2021 Schimpe
#pragma once
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <string>
#include "external/ocam_functions.cpp"

struct OcamModel {
public:
    int width_raw{0}, height_raw{0};
    float center_x{0.0f}, center_y{0.0f};
    struct ocam_model o;

    OcamModel(const std::string &cameraName, const std::string &vehicleID) {
        const std::string nodeName{ros::this_node::getName()};
        const std::string camNs{nodeName + "/" + cameraName};
        // get path to calibration file and load ocam model
        std::string pathToCalib = ros::package::getPath("tod_vehicle_config");
        pathToCalib += "/vehicle_config/" + vehicleID + "/camera-calibration/" + cameraName + ".txt";
        get_ocam_model(&o, pathToCalib.data());
        width_raw = o.width;
        height_raw = o.height;
        center_x = float(o.xc);
        center_y = float(o.yc);
    }

    bool point_on_image(const geometry_msgs::Point &pt, int &x_px, int &y_px,
                        const double scaling_x = 1.0, const double scaling_y = 1.0) const {
        double point3D[3]{pt.y, pt.x, -pt.z};
        double point2D[2]{0.0, 0.0};
        world2cam(point2D, point3D, &o);
        x_px = int(point2D[1]);
        y_px = int(point2D[0]);
        return (1 <= x_px && x_px <= scaling_x * width_raw &&
                1 <= y_px && y_px <= scaling_y * height_raw);
    }
};
