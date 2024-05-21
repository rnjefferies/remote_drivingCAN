// Copyright 2021 Hoffmann
#pragma once
#include <cmath>
#include <fstream>
#include "ros/ros.h"
#include <algorithm>

namespace tod_helper::Files {

inline bool iterate_file(std::string fileName, std::function<void(const std::string &)> callback) {
    std::ifstream in(fileName.c_str());
    if (!in) {
        ROS_ERROR_STREAM("Cannot open the File : "<< fileName);
        return false;
    }
    std::string str;
    while (std::getline(in, str)) {
        callback(str);
    }
    in.close();
    return true;
}
}; // namespace tod_helper::Files
