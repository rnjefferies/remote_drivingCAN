// Copyright 2020 TUMFTM
#pragma once
#include <iostream>
#include <thread>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "Scene/Entity.h"
#include "tod_msgs/KeyPress.h"
#include "geometry_msgs/TransformStamped.h"
class RosInterface {
public:
    RosInterface(int argc, char **pArgv);
    virtual ~RosInterface();

    bool init();
    bool hasParam(const std::string& paramName) { return _nh->hasParam(paramName); }
    void setMouseClickForPublish(const geometry_msgs::Point &mousePosition);
    void setKeyPressForPublish(const tod_msgs::KeyPress &keyiPress);
    static std::string getNodeName();
    static std::string getPackagePath();
    static std::string getPackagePath(std::string path);
    geometry_msgs::TransformStamped tfLookup(const std::string& target_frame, const std::string& source_frame);
    bool debug;

    template <typename T, typename TF, typename... Ts>
    void addSubscriber(const std::string& topicName, TF&& func , Ts... args) {
        if (_nh) {
            _subscriberList.push_back(_nh->subscribe<T>(topicName, 1, boost::bind(func, _1, args...)));
        }
    }

    template<typename T>
    bool getParam(const std::string& paramName, T &retVal) {
        if (!_nh->getParam(paramName, retVal)) {
            ROS_ERROR("%s: could not get param %s", this->getNodeName().c_str(), paramName.c_str());
            return false;
        }
        return true;
    }
    ros::NodeHandle get_node_handle() {
        return *_nh.get();
    }

private:
    std::unique_ptr<ros::NodeHandle> _nh;
    int _initArgc;
    char** _pInitArgv;
    std::thread _rosThread;
    std::vector<ros::Subscriber> _subscriberList;
    ros::Publisher _mouseClickPublisher, _keyPressPublisher;
    tf2_ros::Buffer _tfBuffer;
    bool _newMouseClick{ false };
    bool _newKeyPress{ false };
    tod_msgs::KeyPress _keyPress;
    geometry_msgs::Point _mousePosition;
    void run();
};
