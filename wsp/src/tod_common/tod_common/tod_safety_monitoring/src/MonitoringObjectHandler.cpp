// Copyright 2021 Simon Hoffmann
#include "tod_safety_monitoring/MonitoringObjectHandler.h"

namespace tod_safety_monitoring {

MonitoringObjectHandler::MonitoringObjectHandler(ros::NodeHandle& nh, const std::string& configFile,
        const std::string& namespaceForSafetyState) : _nh(nh) {
    _namespaceForSafetyState = namespaceForSafetyState;
    load_mandatory_topics(configFile);
}

bool MonitoringObjectHandler::init_monitoring_objects() {
    create_monitoring_objects(tod_helper::ROS::get_topic_list("tod_safety_monitoring/SafetyState"));
    return mandatory_objects_initialized();
}

void MonitoringObjectHandler::create_monitoring_objects(const std::vector<std::string>& topics) {
    for (auto topic : topics) {
        if (_topicsToMonitor.find(topic) == _topicsToMonitor.end()) { // no duplicate
            if (_namespaceForSafetyState == topic.substr(0, _namespaceForSafetyState.size())) {
                _topicsToMonitor.insert(topic);
                create_monitoring_object(topic);
            }
        }
    }
}

void MonitoringObjectHandler::create_monitoring_object(std::string& topic) {
    auto object = _monitoringObjects.emplace_back(std::make_shared<MonitoringObject>(topic));
    object->subscriber = _nh.subscribe<tod_safety_monitoring::SafetyState>(topic, 10,
       boost::bind(&MonitoringObjectHandler::callback_safety_status, this, boost::placeholders::_1, object));
    ROS_DEBUG_STREAM("New Subscriber for Topic " << topic << " created");
}

void MonitoringObjectHandler::load_mandatory_topics(const std::string& configFile) {
    if (configFile == "") {
        ROS_DEBUG_STREAM(ros::this_node::getName() << ": No Config File specified!");
        return;
    }
    ROS_DEBUG_STREAM(ros::this_node::getName() << ": Loading mandatoryNodes from: " << configFile);
    tod_helper::Files::iterate_file(configFile, [&](const std::string & str){ _mandatoryTopics.push_back(str); });
}

bool MonitoringObjectHandler::mandatory_objects_initialized() {
    for (const auto& mandatoryTopic : _mandatoryTopics) {
        if (mandatoryTopic == "") continue;
        bool isInitialized = std::any_of(_monitoringObjects.begin(), _monitoringObjects.end(),
            [mandatoryTopic](const auto& obj) {
                return obj->topicName == mandatoryTopic && obj->level > ErrorLvl::DEFAULT;
            });

        if (!isInitialized) {
            ROS_DEBUG_STREAM_THROTTLE(2.0, ros::this_node::getName() << ": " << mandatoryTopic <<
                " is mandatory but not received yet!");
            return false;
        }
    }
    ROS_DEBUG_STREAM(ros::this_node::getName() << ": All Mandatory Objects initialized!");
   return true;
}

ErrorLvl MonitoringObjectHandler::get_current_error_lvl() const noexcept {
    return _currentLvl;
}

void MonitoringObjectHandler::check_timeout() {
    ros::Time now = ros::Time::now();
    for (const auto& obj : _monitoringObjects) {
        if (obj->acceptableTimeout == 0.0) continue;
        if ((now.toSec() - obj->lastStamp.toSec()) > obj->acceptableTimeout) {
            tod_safety_monitoring::SafetyIssue issue;
            issue.errorDescription = "Not heard from " + obj->topicName + " since " +
                std::to_string(now.toSec() - obj->lastStamp.toSec()) + "s (Allowed Timeout: "
                + std::to_string(obj->acceptableTimeout) + " s)";
            issue.errorLevel = static_cast<uint8_t>(ErrorLvl::ERROR);
            issue.errorKey = "SafetyGate Age Check";
            obj->issues.add_issue(issue);
            obj->level = obj->issues.get_max_error_level(); // update only required if issue added
        }
    }
}

void MonitoringObjectHandler::update_error_lvl() noexcept { //overall Level
    _currentLvl = ErrorLvl::OK; //Assume OK first
    for (const auto& object : _monitoringObjects) {
        if (object->level > _currentLvl)
            _currentLvl = object->level;
    }
}

void MonitoringObjectHandler::clear() noexcept {
    for (const auto& obj : _monitoringObjects) {
        obj->issues.clear_buffer();
    }
}

void MonitoringObjectHandler::callback_safety_status(const tod_safety_monitoring::SafetyStateConstPtr& msg,
        std::shared_ptr<MonitoringObject> obj) {
    for (const auto issue : msg->issues) {
        obj->issues.add_issue(issue);
    }
    obj->level = obj->issues.get_max_error_level(); // returns ok if no Issues
    obj->acceptableTimeout = msg->acceptableTimeout;
    obj->lastStamp = ros::Time::now();
}

std::vector<tod_safety_monitoring::SafetyIssue> MonitoringObjectHandler::get_issues_by_level(const ErrorLvl lvl) {
    std::vector<tod_safety_monitoring::SafetyIssue> issues;
    for (const auto& monitoringObject : _monitoringObjects) {
        auto mO_issues = monitoringObject->issues.get_buffer(lvl);
        issues.insert(std::end(issues), std::begin(mO_issues), std::end(mO_issues));
    }
    return issues;
}

void MonitoringObjectHandler::print_issues(const ErrorLvl lvl) {
    IssueBuffer::print_issues(get_issues_by_level(lvl));
}

std::string MonitoringObjectHandler::get_issues_as_string(const ErrorLvl lvl) {
   return IssueBuffer::get_string(get_issues_by_level(lvl));
}
}; // namespace tod_safety_monitoring
