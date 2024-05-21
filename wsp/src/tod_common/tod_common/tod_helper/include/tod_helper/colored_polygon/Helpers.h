// Copyright 2021 Schimpe
#pragma once
#include <geometry_msgs/PolygonStamped.h>
#include <tod_msgs/ColoredPolygon.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tod_helper::ColoredPolygon {

inline static geometry_msgs::PolygonStamped to_polygon(const tod_msgs::ColoredPolygon &coloredPolygon) {
    geometry_msgs::PolygonStamped polygon;
    polygon.header = coloredPolygon.header;
    for (const auto &pt : coloredPolygon.points) {
        polygon.polygon.points.emplace_back(pt.point);
    }
    return polygon;
}

inline tod_msgs::ColoredPolygon to_colored_polygon(const geometry_msgs::PolygonStamped &polygonStamped,
        const std_msgs::ColorRGBA color) {
    tod_msgs::ColoredPolygon coloredPolygon;
    coloredPolygon.header = polygonStamped.header;
    for (const auto &pt : polygonStamped.polygon.points) {
        tod_msgs::ColoredPoint cPoint;
        cPoint.color = color;
        cPoint.point = pt;
        coloredPolygon.points.emplace_back(cPoint);
    }
    return coloredPolygon;
}

inline void transform(tod_msgs::ColoredPolygon &coloredPolygon, const geometry_msgs::TransformStamped &tf) {
    for (tod_msgs::ColoredPoint& cpt : coloredPolygon.points) {
        geometry_msgs::PoseStamped poseInSourceFrame, poseInTargetFrame;
        poseInSourceFrame.header.frame_id = tf.child_frame_id;
        poseInSourceFrame.pose.position.x = cpt.point.x;
        poseInSourceFrame.pose.position.y = cpt.point.y;
        poseInSourceFrame.pose.position.z = cpt.point.z;
        tf2::Quaternion quat(0.0, 0.0, 0.0);
        tf2::convert(quat, poseInSourceFrame.pose.orientation);
        tf2::doTransform<geometry_msgs::PoseStamped>(poseInSourceFrame, poseInTargetFrame, tf);
        cpt.point.x = poseInTargetFrame.pose.position.x;
        cpt.point.y = poseInTargetFrame.pose.position.y;
        cpt.point.z = poseInTargetFrame.pose.position.z;
    }
    coloredPolygon.header.frame_id = tf.header.frame_id;
}

}; // namespace tod_helper::ColoredPolygon
