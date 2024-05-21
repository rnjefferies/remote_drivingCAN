// Copyright 2021 Simon Hoffmann
#pragma once

#include "ros/ros.h"
#include "ros/debug.h"
#include <vector>
#include <map>
#include "tod_safety_monitoring/ErrorLvl.h"
#include <tod_safety_monitoring/SafetyState.h>

namespace tod_safety_monitoring{
class IssueBuffer {
    public:
        void add_issue(tod_safety_monitoring::SafetyIssue issue);
        std::vector<tod_safety_monitoring::SafetyIssue> get_and_clear_buffer();
        std::vector<tod_safety_monitoring::SafetyIssue> get_buffer() const;
        std::vector<tod_safety_monitoring::SafetyIssue> get_buffer(const ErrorLvl& level) const;
        int get_size() const noexcept;
        void clear_buffer() noexcept;

        ErrorLvl get_max_error_level() const noexcept;
        ErrorLvl get_min_error_level() const noexcept;
        static void print_issue(const tod_safety_monitoring::SafetyIssue& issue);
        static void print_issues(const std::vector<tod_safety_monitoring::SafetyIssue>&& issues);
        static std::string get_string(const std::vector<tod_safety_monitoring::SafetyIssue>&& issues) noexcept;
        static std::string get_string(const tod_safety_monitoring::SafetyIssue& issue) noexcept;

    private:
        std::map<ErrorLvl, std::vector<tod_safety_monitoring::SafetyIssue>> _buffer;
        ErrorLvl max_lvl;
        ErrorLvl min_lvl;
};
}; // namespace tod_safety_monitoring

