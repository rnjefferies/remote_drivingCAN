// Copyright 2020 TUMFTM
#include "Core/RosInterface.h"

RosInterface::RosInterface(int argc, char** pArgv)
    :   _initArgc(argc), _pInitArgv(pArgv) { }
RosInterface::~RosInterface() { _rosThread.join(); }

bool RosInterface::init() {
    ros::init(_initArgc, _pInitArgv, "Interface");
    _nh = std::make_unique<ros::NodeHandle>();

    if (!ros::master::check())
        return false;

    _nh->getParam(getNodeName() + "/debug", debug);
    if (debug)
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();

    ros::start();
    ros::Time::init();
    _mouseClickPublisher = _nh->advertise<geometry_msgs::Point>(
        "/Operator/Visual/MousePositionClick", 1);
    _keyPressPublisher = _nh->advertise<tod_msgs::KeyPress>(
        "/Operator/Visual/KeyPress", 1);
    _rosThread = std::thread([this]{ run();});
    ros::Duration(2.0).sleep(); //wait for tf-putlisher to publish

    return true;
}

void RosInterface::run() {
    ros::Rate r(1000);
    tf2_ros::TransformListener _tfListener(_tfBuffer);
    while ( ros::ok() ) {
        ros::spinOnce();
        if ( _newMouseClick ) {
            _mouseClickPublisher.publish(_mousePosition);
            _newMouseClick = false;
        }
        if ( _newKeyPress ) {
            _keyPressPublisher.publish(_keyPress);
            _newKeyPress = false;
        }
        r.sleep();
    }
}

std::string RosInterface::getNodeName() { return ros::this_node::getName(); }
std::string RosInterface::getPackagePath() { return ros::package::getPath("tod_visual"); }

std::string RosInterface::getPackagePath(std::string path) {
   return ros::package::getPath(path);
}

geometry_msgs::TransformStamped RosInterface::tfLookup(
    const std::string& target_frame, const std::string& source_frame) {
    geometry_msgs::TransformStamped tf;
    try {
        tf = _tfBuffer.lookupTransform(target_frame, source_frame, ros::Time::now());
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s: Look up transform for %s failed with %s", this->getNodeName().c_str(),
                  source_frame.c_str(), ex.what());
    }
    return tf;
}

void RosInterface::setMouseClickForPublish(const geometry_msgs::Point &mousePosition) {
    _mousePosition = mousePosition;
    _newMouseClick = true;
}

void RosInterface::setKeyPressForPublish(const tod_msgs::KeyPress &keyPress) {
    _keyPress = keyPress;
    _keyPress.header.stamp = ros::Time::now();
    _newKeyPress = true;
}
