// Copyright 2021 Simon Hoffmann
#include "ros/ros.h"
#include "ros/debug.h"
#include "tod_safety_monitoring/SafetyState.h"
#include "tod_safety_monitoring/SafetyMonitor.h"
#include "geometry_msgs/PolygonStamped.h"
#include "memory.h"
#include "tod_msgs/VehicleData.h"
#include "tod_safety_monitoring/ErrorLvl.h"
#include "tod_msgs/Trajectory.h"

std::unique_ptr<tod_safety_monitoring::SafetyMonitor> monitor;

std::shared_ptr<tod_safety_monitoring::MaxDurationTest> durationCheckVehData;
std::shared_ptr<tod_safety_monitoring::MaxDurationTest> durationCheckTrajectory;

void veh_data_callback(const tod_msgs::VehicleDataConstPtr msg) { // WARNING
    durationCheckVehData->update();
}

void trajectory_callback(const tod_msgs::TrajectoryConstPtr msg) { // WARNING
    durationCheckTrajectory->update();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleWatchdog");
    ros::NodeHandle nh;
    ros::Rate r(100);
    monitor = std::make_unique<tod_safety_monitoring::SafetyMonitor>(nh);
    monitor->acceptable_timeout(0.5); // if node_status not published for 500ms -> Gate closed
    ros::Subscriber vehDataSubs = nh.subscribe("/Vehicle/VehicleBridge/vehicle_data", 5, veh_data_callback);
    ros::Subscriber trajectorySubs = nh.subscribe("/Vehicle/SafeCorridorControl/trajectory_control_command",
        5, trajectory_callback);

    durationCheckVehData = std::make_shared<tod_safety_monitoring::MaxDurationTest>(0.4,
        static_cast<tod_safety_monitoring::ErrorLvl>(2), "VehData", "Duration Test exceeded");
    monitor->add_periodic_test(durationCheckVehData);

    durationCheckTrajectory = std::make_shared<tod_safety_monitoring::MaxDurationTest>(0.2,
        static_cast<tod_safety_monitoring::ErrorLvl>(3), "VehData", "Duration Test exceeded");
    monitor->add_periodic_test(durationCheckTrajectory);

    // Control Queue implicitly handeled
    while (ros::ok()) {
        monitor->on_update(); // run tests and publish results
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
