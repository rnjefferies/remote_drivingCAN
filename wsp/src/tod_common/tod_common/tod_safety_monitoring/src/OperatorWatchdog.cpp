// Copyright 2021 Simon Hoffmann
#include "ros/ros.h"
#include "ros/debug.h"
#include "tod_safety_monitoring/SafetyState.h"
#include "tod_safety_monitoring/ErrorLvl.h"
#include "tod_safety_monitoring/SafetyMonitor.h"
#include "tod_msgs/ColoredPolygon.h"
#include "memory.h"
#include "tod_msgs/VehicleData.h"
#include "tod_msgs/SecondaryControlCmd.h"
#include "sensor_msgs/Image.h"

std::unique_ptr<tod_safety_monitoring::SafetyMonitor> monitor;
std::shared_ptr<tod_safety_monitoring::MaxDurationTest> durationCheckCorridor;
std::shared_ptr<tod_safety_monitoring::MaxDurationTest> durationCheckCenterCam;
std::shared_ptr<tod_safety_monitoring::MaxDurationTest> durationCheckLeftCam;
std::shared_ptr<tod_safety_monitoring::MaxDurationTest> durationCheckRightCam;
std::shared_ptr<tod_safety_monitoring::MaxDurationTest> durationCheckVehData;


void corr_callback(const tod_msgs::ColoredPolygonConstPtr& msg) {
    durationCheckCorridor->update();
}

void cam_center_callback(const sensor_msgs::ImageConstPtr& msg) {
    durationCheckCenterCam->update();
}

void cam_left_callback(const sensor_msgs::ImageConstPtr& msg) {
    durationCheckLeftCam->update();
    //Add Data Quality Checks for Cameras
}

void cam_right_callback(const sensor_msgs::ImageConstPtr& msg) {
    durationCheckRightCam->update();
}

void veh_data_callback(const tod_msgs::VehicleDataConstPtr& msg) { // WARNING
    durationCheckVehData->update();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "OperatorWatchdog");
    ros::NodeHandle nh;
    ros::Rate r(100);
    monitor = std::make_unique<tod_safety_monitoring::SafetyMonitor>(nh);
    monitor->acceptable_timeout(0.5); // if node_status not published for 500ms -> Gate closed
    ros::Subscriber corrSubs = nh.subscribe("/Operator/SafeCorridorControl/corridor", 5, corr_callback);
    ros::Subscriber camCenterSubs = nh.subscribe("/Operator/Video/CameraFrontCenter/image_raw", 5, cam_center_callback);
    ros::Subscriber camRightSubs = nh.subscribe("/Operator/Video/CameraFrontRight/image_raw", 5, cam_left_callback);
    ros::Subscriber camLeftSubs = nh.subscribe("/Operator/Video/CameraFrontLeft/image_raw", 5, cam_right_callback);
    ros::Subscriber vehDataSubs = nh.subscribe("/Operator/VehicleBridge/vehicle_data", 5, veh_data_callback);

    durationCheckCorridor = std::make_shared<tod_safety_monitoring::MaxDurationTest>(0.1,
        static_cast<tod_safety_monitoring::ErrorLvl>(3), "CorridorCallback", "Duration Test exceeded");
    monitor->add_periodic_test(durationCheckCorridor);

    durationCheckCenterCam = std::make_shared<tod_safety_monitoring::MaxDurationTest>(0.2,
        static_cast<tod_safety_monitoring::ErrorLvl>(3), "CenterCam", "Duration Test exceeded");
    monitor->add_periodic_test(durationCheckCenterCam);

    durationCheckLeftCam = std::make_shared<tod_safety_monitoring::MaxDurationTest>(0.2,
        static_cast<tod_safety_monitoring::ErrorLvl>(3), "LeftCam", "Duration Test exceeded");
    monitor->add_periodic_test(durationCheckLeftCam);

    durationCheckRightCam = std::make_shared<tod_safety_monitoring::MaxDurationTest>(0.2,
        static_cast<tod_safety_monitoring::ErrorLvl>(3), "RightCam", "Duration Test exceeded");
    monitor->add_periodic_test(durationCheckRightCam);

    durationCheckVehData = std::make_shared<tod_safety_monitoring::MaxDurationTest>(0.4,
        static_cast<tod_safety_monitoring::ErrorLvl>(2), "VehData", "Duration Test exceeded");
    monitor->add_periodic_test(durationCheckVehData);

    while (ros::ok()) {
        monitor->on_update(); // run tests and publish results
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
