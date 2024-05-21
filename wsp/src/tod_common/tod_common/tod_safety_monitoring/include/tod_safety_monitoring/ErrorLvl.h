// Copyright 2021 Simon Hoffmann
#pragma once
#include <iostream>
#include <string>

namespace tod_safety_monitoring {

enum class ErrorLvl {
    DEFAULT = 0,
    OK = 1,
    WARNING = 2,
    ERROR = 3
};

inline std::ostream &operator<<(std::ostream &os, const ErrorLvl &e) {
    switch (e) {
        case ErrorLvl::DEFAULT: os << "DEFAULT"; break;
        case ErrorLvl::OK: os << "OK"; break;
        case ErrorLvl::WARNING: os << "WARNING"; break;
        case ErrorLvl::ERROR: os << "ERROR"; break;
        default: os << "UNDEFINED ERROR LEVEL";
    }
    return os;
}

std::string color_error_lvl(const ErrorLvl &e);
}; // namespace tod_safety_monitoring
