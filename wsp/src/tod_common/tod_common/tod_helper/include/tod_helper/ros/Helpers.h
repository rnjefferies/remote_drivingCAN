// Copyright 2021 Hoffmann
#pragma once
#include "ros/ros.h"

namespace tod_helper::ROS {

inline std::vector<std::string> get_topic_list(const std::string& msg_type) {
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);
    std::vector<std::string> topicList;
    for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++) {
        if (it->datatype == msg_type) {
            topicList.push_back(it->name);
        }
    }
    return topicList;
}

}; // namespace tod_helper::ROS
