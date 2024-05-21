
// Copyright 2020 Simon Hoffmann
#include "ros/ros.h"
#include "ros/debug.h"
#include "tod_safety_monitoring/SafetyGate.h"
#include <tod_safety_monitoring/SafetyState.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "SafetyGate");
    ros::NodeHandle nh;
    ros::Rate r(100);
    SafetyGate gate(nh);
    gate.run();
    return 0;
}
