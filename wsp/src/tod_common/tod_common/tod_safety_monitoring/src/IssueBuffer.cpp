// Copyright 2021 Simon Hoffmann
#include "tod_safety_monitoring/IssueBuffer.h"
#include "tod_safety_monitoring/SafetyMonitor.h"

namespace tod_safety_monitoring {
void IssueBuffer::add_issue(tod_safety_monitoring::SafetyIssue issue) {
    _buffer[static_cast<ErrorLvl>(issue.errorLevel)].push_back(issue);
    max_lvl = ErrorLvl::ERROR;
    min_lvl = ErrorLvl::OK;
}

std::vector<tod_safety_monitoring::SafetyIssue> IssueBuffer::get_and_clear_buffer() {
    std::vector<tod_safety_monitoring::SafetyIssue> status;
    status = get_buffer();
    _buffer.clear();
   return status;
}

std::vector<tod_safety_monitoring::SafetyIssue> IssueBuffer::get_buffer() const {
    std::vector<tod_safety_monitoring::SafetyIssue> status;
    for (auto it = _buffer.begin(); it != _buffer.end(); ++it) {
        status.insert(status.end(), it->second.begin(), it->second.end());
    }
   return status;
}

std::vector<tod_safety_monitoring::SafetyIssue> IssueBuffer::get_buffer(const ErrorLvl& level) const {
    return _buffer.at(level);
}

int IssueBuffer::get_size() const noexcept {
    return get_buffer().size();
}

void IssueBuffer::clear_buffer() noexcept {
    _buffer.clear();
}

ErrorLvl IssueBuffer::get_max_error_level() const noexcept {
    ErrorLvl out{ErrorLvl::OK};
    for (auto const& issues : _buffer) {
        if (!issues.second.empty() && issues.first >= out) {
            out = issues.first;
        }
    }
    return out;
}

ErrorLvl IssueBuffer::get_min_error_level() const noexcept {
    ErrorLvl out{99};
    for (auto const& issues : _buffer) {
        if (!issues.second.empty() && issues.first <= out) {
            out = issues.first;
        }
    }
    return out;
}

void IssueBuffer::print_issue(const tod_safety_monitoring::SafetyIssue& issue) {
    std::cerr << color_error_lvl(static_cast<ErrorLvl>(issue.errorLevel)) <<
        "[" << static_cast<ErrorLvl>(issue.errorLevel) << " - Safety Monitor] "
        <<  issue.parentNode << " (" << issue.errorKey << "): "
        << issue.errorDescription << "\033[49m" << "\n";
}

void IssueBuffer::print_issues(const std::vector<tod_safety_monitoring::SafetyIssue>&& issues) {
    for (const auto& obj : issues) {
        print_issue(obj);
    }
}

std::string IssueBuffer::get_string(const std::vector<tod_safety_monitoring::SafetyIssue>&& issues) noexcept {
    std::string issue_string;
    for (const auto& obj : issues) {
        issue_string += get_string(obj);
    }
    return issue_string;
}

std::string IssueBuffer::get_string(const tod_safety_monitoring::SafetyIssue& issue) noexcept {
    std::string issue_string;
    issue_string = "[Safety Monitor] " +  issue.parentNode + " (" + issue.errorKey + "): " +
        issue.errorDescription + "\n";
    return issue_string;
}
}; // namespace tod_safety_monitoring
