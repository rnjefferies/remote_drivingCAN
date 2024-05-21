// Copyright 2021 Simon Hoffmann
#pragma once

namespace tod_safety_monitoring {

class SafetyCheck {
public:
    SafetyCheck() = default;
    SafetyCheck(const SafetyCheck&) = default;
    SafetyCheck(SafetyCheck&&) = default;
    SafetyCheck& operator=(const SafetyCheck&) = default;
    SafetyCheck& operator=(SafetyCheck&&) = default;
    virtual ~SafetyCheck() = default;
    virtual bool successfull() const = 0;

    ErrorLvl _lvl {1};
    std::string _key{""};
    std::string _description{""};
};

struct MinValueTest : SafetyCheck {
public:
    MinValueTest(const double min_val, const double val, const ErrorLvl level,
        const std::string& key, const std::string& description) :
            _val(val), _min_val(min_val) {
                _lvl = level;
                _key = key;
                _description = description;
            }
private:
    double _min_val{0}; // template double away
    double _val{0};
    bool successfull() const override {
        if (_val >= _min_val)
            return false;
        return true;
    };
};

struct MaxDurationTest : SafetyCheck {
public:
    MaxDurationTest(const double max_duration_s, const ErrorLvl level,
            const std::string& key, const std::string& description) {
        _maxDuration = ros::Duration(max_duration_s);
        _lvl = level;
        _key = key;
        _description = description;
    }
    void update() {
        _firstUpdateCall = true;
        _timeLastUpdate = ros::Time::now();
    }

private:
    ros::Duration _maxDuration;
    ros::Time _timeLastUpdate;
    bool _firstUpdateCall{false};

    bool successfull() const override {
        if (!_firstUpdateCall)// Measure time between updates;
            return true;
        if (_maxDuration <= ros::Time::now() - _timeLastUpdate )
            return false;
        return true;
    };
};
}; //namespace tod_safety_monitoring
