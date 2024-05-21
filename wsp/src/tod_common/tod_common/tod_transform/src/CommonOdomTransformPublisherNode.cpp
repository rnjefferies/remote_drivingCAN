// Copyright 2020 Feiler
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/static_transform_broadcaster.h"

geometry_msgs::TransformStamped createTransformFromOdomMsg(const nav_msgs::Odometry &msg);
void odomMessageReveived(const nav_msgs::Odometry &msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "CommonOdomTransformPublisher");
    ros::NodeHandle n;
    ros::Subscriber vehDataSubs = n.subscribe("odometry", 1, odomMessageReveived);
    ros::spin();
    return 0;
}

void odomMessageReveived(const nav_msgs::Odometry &msg) {
    static tf2_ros::StaticTransformBroadcaster odom_broadcaster;
    ROS_INFO_ONCE("%s: Broadcasing odometry transform from %s to %s",
                  ros::this_node::getName().c_str(),
                  msg.header.frame_id.c_str(),
                  msg.child_frame_id.c_str());
    geometry_msgs::TransformStamped transfrom = createTransformFromOdomMsg(msg);
    odom_broadcaster.sendTransform(transfrom);
}

geometry_msgs::TransformStamped createTransformFromOdomMsg(
    const nav_msgs::Odometry &msg) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = msg.header.frame_id;
    transform.child_frame_id = msg.child_frame_id;
    transform.transform.translation.x = msg.pose.pose.position.x;
    transform.transform.translation.y = msg.pose.pose.position.y;
    transform.transform.translation.z = msg.pose.pose.position.z;
    transform.transform.rotation = msg.pose.pose.orientation;
    return transform;
}
