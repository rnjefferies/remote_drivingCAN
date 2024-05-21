// Copyright 2021 Schimpe
#include <ros/ros.h>
#include <tod_msgs/Trajectory.h>
#include <nav_msgs/Path.h>
#include <tod_helper/trajectory/Helpers.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "TrajectoryViz");
    ros::NodeHandle nh;
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("path", 1);
    bool zCoordAsDesiredVelocity{false};
    nh.getParam(std::string(ros::this_node::getName() + "/z_coordinate_as_desired_velocity"), zCoordAsDesiredVelocity);
    ros::Subscriber subTrajectory = nh.subscribe<tod_msgs::Trajectory>(
        "trajectory", 1, [&](const tod_msgs::TrajectoryConstPtr &msg) {
            nav_msgs::Path path = tod_helper::Trajectory::to_path(*msg, zCoordAsDesiredVelocity);
            pubPath.publish(path);
        });
    ros::spin();
    return 0;
}
