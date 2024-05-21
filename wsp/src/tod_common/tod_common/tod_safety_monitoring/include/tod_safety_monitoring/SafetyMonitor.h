// Copyright 2021 Simon Hoffmann
#pragma once

#include "ros/ros.h"
#include "ros/debug.h"
#include <vector>
#include <map>
#include <thread>
#include <tod_safety_monitoring/SafetyState.h>
#include <tod_safety_monitoring/IssueBuffer.h>
#include <tod_safety_monitoring/SafetyChecks.h>

namespace tod_safety_monitoring {
class SafetyMonitor{
    public:
        explicit SafetyMonitor(ros::NodeHandle &nh);
        void on_update();
        void perform_test_once(const SafetyCheck&& check);
        void perform_test_once(const SafetyCheck& check);
        void add_safety_issue(const tod_safety_monitoring::SafetyIssue& issue);
        void add_periodic_test(std::shared_ptr<SafetyCheck> check);
        void acceptable_timeout(const float sec);
        int get_number_by_level(const ErrorLvl level) const;
        int get_number_of_issues() const;
        ErrorLvl get_min_error_lvl() { return _issueBuffer->get_min_error_level(); }
        ErrorLvl get_max_error_lvl() { return _issueBuffer->get_max_error_level(); }
        std::vector<tod_safety_monitoring::SafetyIssue> get_all_issues();

    private:
        ros::NodeHandle _nh;
        std::string _nodeName;
        void perform_all_tests();
        ros::Publisher _statusPub;
        float _acceptable_timeout{0.0};
        std::unique_ptr<IssueBuffer> _issueBuffer;
        std::vector<std::shared_ptr<SafetyCheck>> _safetyChecks;
        void perform_test_once(const SafetyCheck* check);
};
}; //namespace tod_safety_monitoring
