// Copyright 2020 Feiler
#include <ros/ros.h>
#include <QApplication>
#include "OperatorManager.h"
#include <ros/package.h>

int main(int argc, char **argv) {
    QApplication a(argc, argv);
    ros::init(argc, argv, "OperatorManager");
    std::string pathToYamlFile = ros::package::getPath("tod_manager") + "/yaml/InitialIpAddresses.yaml";
    OperatorManager operatorManager(pathToYamlFile, "OperatorManagerVehicleIpAddresses");
    operatorManager.create_and_run_ros_thread();
    return a.exec();
}
