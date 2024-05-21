// Copyright 2021 Schimpe
#include <ros/ros.h>
#include <tod_msgs/ObjectList.h>
#include <visualization_msgs/MarkerArray.h>
#include <tod_helper/object_list/Helpers.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ObjectListViz");
    ros::NodeHandle nh;
    ros::Publisher pubMarker = nh.advertise<visualization_msgs::MarkerArray>("object_marker", 100);
    ros::Subscriber subTrajectory = nh.subscribe<tod_msgs::ObjectList>(
        "object_list", 1, [&](const tod_msgs::ObjectListConstPtr &msg) {
            visualization_msgs::MarkerArray markerArray = tod_helper::ObjectList::to_marker_array(*msg);
            pubMarker.publish(markerArray);
        });
    ros::spin();
    return 0;
}
