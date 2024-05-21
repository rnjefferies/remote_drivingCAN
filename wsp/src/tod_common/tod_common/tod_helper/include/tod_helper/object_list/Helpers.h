// Copyright 2020 Andreas Schimpe
#pragma once
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <tod_msgs/ObjectList.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tod_helper::ObjectList {

inline std::vector<visualization_msgs::Marker> to_marker_vector(const std::vector <tod_msgs::ObjectData> &objects) {
    std::vector<visualization_msgs::Marker> markers;
    for (int i=0; i < objects.size(); ++i) {
        auto &cube = markers.emplace_back(visualization_msgs::Marker());
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, objects.at(i).yawAngle);
        cube.id = i;
        cube.type = visualization_msgs::Marker::CUBE;
        cube.scale.x = 0.1;
        cube.color.r = 1.0;
        cube.color.a = 1.0;
        cube.pose.orientation.x = quaternion.x();
        cube.pose.orientation.y = quaternion.y();
        cube.pose.orientation.z = quaternion.z();
        cube.pose.orientation.w = quaternion.w();
        cube.pose.position.x = objects.at(i).distCenterX;
        cube.pose.position.y = objects.at(i).distCenterY;
        cube.scale.x = objects.at(i).dimX;
        cube.scale.y = objects.at(i).dimY;
        cube.scale.z = objects.at(i).dimZ;
        cube.pose.position.z = -0.5 * cube.scale.z;
        cube.lifetime = ros::Duration(0.105);
    }
    return markers;
}

inline visualization_msgs::MarkerArray to_marker_array(const tod_msgs::ObjectList &objectList) {
    visualization_msgs::MarkerArray markerArray;
    markerArray.markers = to_marker_vector(objectList.objectList);
    for (auto &marker : markerArray.markers) {
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = objectList.header.frame_id;
    }
    return markerArray;
}

inline void transform(std::vector<tod_msgs::ObjectData> &objects, geometry_msgs::TransformStamped &tf) {
    for (tod_msgs::ObjectData &object : objects) {
        geometry_msgs::PoseStamped poseInSourceFrame, poseInTargetFrame;
        poseInSourceFrame.header.frame_id = tf.child_frame_id;
        poseInSourceFrame.pose.position.x = object.distCenterX;
        poseInSourceFrame.pose.position.y = object.distCenterY;
        poseInSourceFrame.pose.position.z = 0.0;
        tf2::Quaternion quat(object.yawAngle, 0.0, 0.0);
        tf2::convert(quat, poseInSourceFrame.pose.orientation);
        tf2::doTransform<geometry_msgs::PoseStamped>(poseInSourceFrame, poseInTargetFrame, tf);
        object.distCenterX = float(poseInTargetFrame.pose.position.x);
        object.distCenterY = float(poseInTargetFrame.pose.position.y);
        tf2::convert(poseInTargetFrame.pose.orientation, quat);
        tf2::Matrix3x3 eulerMat(quat);
        double r, p, y;
        eulerMat.getRPY(r, p, y);
        object.yawAngle = float(y);
    }
}

}; //namespace tod_helper::ObjectList
