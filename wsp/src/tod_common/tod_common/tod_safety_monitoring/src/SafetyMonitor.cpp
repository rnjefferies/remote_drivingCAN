// Copyright 2021 Simon Hoffmann
#include "tod_safety_monitoring/SafetyMonitor.h"

namespace tod_safety_monitoring {

SafetyMonitor::SafetyMonitor(ros::NodeHandle &nh) : _nh(nh) {
    _nodeName = ros::this_node::getName();
    _statusPub = _nh.advertise<tod_safety_monitoring::SafetyState>(_nodeName + "/node_status", 5);
    _issueBuffer = std::make_unique<IssueBuffer>();
    ROS_DEBUG_STREAM("SafetyMonitor for Node " << ros::this_node::getName() << " started!");
}

void SafetyMonitor::on_update() {
    tod_safety_monitoring::SafetyState status;
    status.nodeName = _nodeName;
    status.acceptableTimeout = _acceptable_timeout;
    perform_all_tests();
    status.issues = _issueBuffer->get_and_clear_buffer();
    _statusPub.publish(status);
}

void SafetyMonitor::acceptable_timeout(const float sec) {
   _acceptable_timeout = sec;
}

void SafetyMonitor::perform_all_tests() {
    for (const auto& test : _safetyChecks) {
        perform_test_once(test.get());
    }
}

void SafetyMonitor::perform_test_once(const SafetyCheck&& check) {
    perform_test_once(&check);
}

void SafetyMonitor::perform_test_once(const SafetyCheck& check) {
    perform_test_once(&check);
}

void SafetyMonitor::perform_test_once(const SafetyCheck* check) {
    if (!check) { return; }
    if ( !check->successfull() ) {
        tod_safety_monitoring::SafetyIssue issue;
        issue.header.stamp = ros::Time::now();
        issue.errorDescription = check->_description;
        issue.errorLevel = static_cast<uint8_t>(check->_lvl);
        issue.parentNode = _nodeName;
        issue.errorKey = check->_key;
        add_safety_issue(issue);
    }
}

void SafetyMonitor::add_periodic_test(std::shared_ptr<SafetyCheck> check) {
    _safetyChecks.push_back(check);
}

void SafetyMonitor::add_safety_issue(const tod_safety_monitoring::SafetyIssue& issue) {
    _issueBuffer->add_issue(issue);
}

int SafetyMonitor::get_number_by_level(const ErrorLvl level) const {
    return _issueBuffer->get_buffer(level).size();
}

int SafetyMonitor::get_number_of_issues() const{
    return _issueBuffer->get_buffer().size();
}
}; // namespace tod_safety_monitoring
