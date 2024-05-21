// Copyright 2020 Hoffmann
#pragma once
#include <cmath>
#include "geometry_msgs/Point.h"
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

namespace tod_helper::Geometry {
inline double calc_horizontal_distance(const geometry_msgs::Point &pt0, const geometry_msgs::Point &pt1) {
    return std::sqrt(std::pow((pt0.x - pt1.x), 2) + std::pow((pt0.y - pt1.y), 2));
}

// From: https://github.com/Autoware-AI/common/blob/master/libwaypoint_follower/src/libwaypoint_follower.cpp
inline geometry_msgs::Point calc_relative_position(geometry_msgs::Point point_msg,
        geometry_msgs::Pose current_pose) {
    tf::Transform inverse;
    tf::poseMsgToTF(current_pose, inverse);
    tf::Transform transform = inverse.inverse();

    tf::Point p;
    pointMsgToTF(point_msg, p);
    tf::Point tf_p = transform * p;
    geometry_msgs::Point tf_point_msg;
    pointTFToMsg(tf_p, tf_point_msg);

    return tf_point_msg;
}

inline static double get_yaw_from_quaternion(const geometry_msgs::Quaternion& orientation) {
    return tf2::getYaw(orientation);
}

inline static double perpendicular_from_pt_on_line(
    const geometry_msgs::Point &pt, const geometry_msgs::Point &line0,
    const geometry_msgs::Point &line1, geometry_msgs::Point &perpendicular) {
    double x0 = line0.x; double y0 = line0.y;
    double x1 = line1.x; double y1 = line1.y;
    double x2 = pt.x; double y2 = pt.y;
    // first convert line to normalized unit vector
    double dx = x1 - x0;
    double dy = y1 - y0;
    double mag = sqrt(dx*dx + dy*dy);
    dx /= mag;
    dy /= mag;
    // translate the point and get the dot product
    double lambda = (dx * (x2 - x0)) + (dy * (y2 - y0));
    perpendicular.x = (dx * lambda) + x0;
    perpendicular.y = (dy * lambda) + y0;
    return lambda;
}

}; // namespace tod_helper::Geometry
