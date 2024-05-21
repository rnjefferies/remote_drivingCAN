// Copyright 2021 Simon Hoffmann
#pragma once
#include "iostream"
#include "Definitions.h"
#include "std_msgs/ColorRGBA.h"

namespace tod_safety_monitoring {

inline std::ostream &operator<<(std::ostream &os, const Event &e) {
    switch (e) {
        case Event::OPEN: os << "open"; break;
        case Event::RESET: os << "reset"; break;
        case Event::CLOSE: os << "close"; break;
        case Event::WARN: os << "warn"; break;
        case Event::INITIALIZED: os << "initialized"; break;
    }
    return os;
}

inline std_msgs::ColorRGBA get_color(StateMachineState state) {
    std_msgs::ColorRGBA color;
    color.a = 0.4;
    switch (state) {
        case StateMachineState::OPEN:
            color.g = 1.0;
            break;
        case StateMachineState::WARNING:
            color.r = 1.0; color.g = 1.0;
            break;
        case StateMachineState::CLOSED:
            color.r = 1.0;
            break;
        default:
            color.r = 0.5; color.b = 0.5; color.g = 0.5;
    }
    return color;
}

inline std::ostream &operator<<(std::ostream &os, const StateMachineState &state) {
    switch (state) {
        case StateMachineState::STARTUP: os << "Startup"; break;
        case StateMachineState::INITIALIZED: os << "Initialized"; break;
        case StateMachineState::OPEN: os << "Open"; break;
        case StateMachineState::CLOSED: os << "Closed"; break;
        case StateMachineState::WARNING: os << "Warning"; break;
        default: os << "Undefined";
    }
    return os;
}
}; // namespace tod_safety_monitoring
