// Copyright 2021 Schimpe
#include <ros/ros.h>
#include <tod_msgs/ColoredPolygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tod_helper/colored_polygon/Helpers.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ColoredPolygonViz");
    ros::NodeHandle nh;
    ros::Publisher pubPolygon = nh.advertise<geometry_msgs::PolygonStamped>("polygon", 1);
    ros::Subscriber subColoredPolygon = nh.subscribe<tod_msgs::ColoredPolygon>(
        "colored_polygon", 1, [&](const tod_msgs::ColoredPolygonConstPtr &msg) {
            geometry_msgs::PolygonStamped polygon = tod_helper::ColoredPolygon::to_polygon(*msg);
            pubPolygon.publish(polygon);
        });
    ros::spin();
    return 0;
}
