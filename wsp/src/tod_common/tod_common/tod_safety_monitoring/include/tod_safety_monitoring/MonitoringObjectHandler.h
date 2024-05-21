// Copyright 2021 Simon Hoffmann
#pragma once
#include "ros/ros.h"
#include <string.h>
#include "tod_safety_monitoring/SafetyMonitor.h"
#include "tod_safety_monitoring/SafetyState.h"
#include "tod_helper/files/Helpers.h"
#include "tod_helper/ros/Helpers.h"
#include <algorithm>

namespace tod_safety_monitoring {
struct MonitoringObject {
    explicit MonitoringObject(const std::string& topic) { topicName = topic; }
    ros::Subscriber subscriber;
    ros::Time lastStamp;
    float acceptableTimeout{0.0};
    std::string topicName;
    ErrorLvl level{ErrorLvl::DEFAULT}; // highest level in received msgs
    IssueBuffer issues;
};

class MonitoringObjectHandler {
public:
    MonitoringObjectHandler(ros::NodeHandle &nh, const std::string& configFile,
        const std::string& namespaceForSafetyState = "");
    ErrorLvl get_current_error_lvl() const noexcept;
    std::vector<tod_safety_monitoring::SafetyIssue> get_issues_by_level(const ErrorLvl lvl);
    std::string get_issues_as_string(const ErrorLvl lvl);
    bool init_monitoring_objects();
    bool mandatory_objects_initialized();
    void create_monitoring_objects(const std::vector<std::string>& topics);
    void create_monitoring_object(std::string& topic);
    void check_timeout();
    void update_error_lvl() noexcept;
    void clear() noexcept;
    void print_issues(const ErrorLvl lvl);

private:
    ros::NodeHandle _nh;
    std::vector<std::shared_ptr<MonitoringObject>> _monitoringObjects;
    std::set<std::string> _topicsToMonitor;
    std::vector<std::string> _mandatoryTopics;
    std::string _namespaceForSafetyState{""};
    ErrorLvl _currentLvl{ErrorLvl::DEFAULT}; // MAX over all Monitoring Objects
    void load_mandatory_topics(const std::string &configFile);
    void callback_safety_status(const tod_safety_monitoring::SafetyStateConstPtr& msg,
        std::shared_ptr<MonitoringObject> obj);
};
}; // namespace tod_safety_monitoring
