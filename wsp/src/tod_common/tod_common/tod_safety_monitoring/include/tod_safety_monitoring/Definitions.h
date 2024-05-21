// Copyright 2021 Simon Hoffmann
#pragma once
namespace tod_safety_monitoring {
enum class Event {
    OPEN,
    RESET,
    CLOSE,
    WARN,
    INITIALIZED
};

enum class StateMachineState {
    UNDEFINED = 0,
    STARTUP = 1,
    INITIALIZED = 2,
    OPEN = 3,
    WARNING = 4,
    CLOSED = 5
};
}; // namespace tod_safety_monitoring
