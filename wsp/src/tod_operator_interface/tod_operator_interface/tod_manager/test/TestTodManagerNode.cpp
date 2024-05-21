// Copyright 2020 Schimpe

#include <ros/ros.h>
#include "tod_msgs/Status.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "TestTodManager");
    ros::NodeHandle n;

    std::string topicVehicleConnStatus{"/Vehicle/Manager/status_msg"};
    ros::Publisher pubVehicleConnStatus = n.advertise<tod_msgs::Status>(topicVehicleConnStatus, 5);

    std::string topicOperatorConnStatus{"/Operator/Manager/status_msg"};
    ros::Publisher pubOperatorConnStatus = n.advertise<tod_msgs::Status>(topicOperatorConnStatus, 5);

    tod_msgs::Status msg;
    msg.operator_broker_ip_address = msg.vehicle_vehicle_ip_address =
            msg.operator_operator_ip_address = "127.0.0.1";
    msg.vehicle_lat_approved = msg.vehicle_long_approved =
            msg.vehicle_emergency_stop_released = false;
    msg.vehicle_control_mode = msg.operator_control_mode =
            tod_msgs::Status::CONTROL_MODE_DIRECT;
    msg.tod_status = tod_msgs::Status::TOD_STATUS_IDLE;

    ros::Rate r(100);
    ros::Time t = ros::Time::now();
    while (ros::ok()) {
        r.sleep();

        if (ros::Time::now() >= t + ros::Duration(2.0)) {
            msg.tod_status = tod_msgs::Status::TOD_STATUS_UPLINK_ONLY;
        }

        pubVehicleConnStatus.publish(msg);
        pubOperatorConnStatus.publish(msg);
        ros::spinOnce();
    }

    return 0;
}
