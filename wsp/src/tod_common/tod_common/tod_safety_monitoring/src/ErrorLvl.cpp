// Copyright 2021 Simon Hoffmann
#include <iostream>
#include <string>
#include "tod_safety_monitoring/ErrorLvl.h"

namespace tod_safety_monitoring {
std::string color_error_lvl(const ErrorLvl &e) {
    std::string os;
    switch (e) {
        case ErrorLvl::OK: os = "\033[49m"; break;
        case ErrorLvl::WARNING: os = "\033[33m"; break;
        case ErrorLvl::ERROR: os = "\033[31m"; break;
    }
    return os;
}
}; // namespace tod_safety_monitoring
