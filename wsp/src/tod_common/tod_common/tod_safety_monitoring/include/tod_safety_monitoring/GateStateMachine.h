// Copyright 2021 Simon Hoffmann
#pragma once

#include "ros/ros.h"
#include "ros/debug.h"
#include <vector>
#include <map>
#include <thread>
#include <iostream>
#include <variant>
#include <optional>
#include "tod_msgs/Trajectory.h"
#include "tod_safety_monitoring/Definitions.h"

namespace tod_safety_monitoring {
class BaseGate;

class State {
    public:
        virtual void on_update(BaseGate* _gatePtr) {}
        virtual std::unique_ptr<State> on_event(Event e) = 0;
        friend std::ostream& operator<<(std::ostream &os, const std::unique_ptr<State> &state);
        StateMachineState state{StateMachineState::UNDEFINED};
};

/* ---------------------------------  States ------------------------------------------ */
class Startup : public State {
public:
    Startup() { this->state = StateMachineState::STARTUP; }
    std::unique_ptr<State> on_event(Event e) override;
    void on_update(BaseGate* gatePtr) override;
};

class Initialized : public State {
public:
    Initialized() { this->state = StateMachineState::INITIALIZED; }
    std::unique_ptr<State> on_event(Event e) override;
    void on_update(BaseGate* gatePtr) override;
};

class Closed : public State {
public:
    Closed() { this->state = StateMachineState::CLOSED; }
    std::unique_ptr<State> on_event(Event e) override;
    void on_update(BaseGate* gatePtr) override;
};

class Open : public State {
public:
    Open() { this->state = StateMachineState::OPEN; }
    std::unique_ptr<State> on_event(Event e) override;
    void on_update(BaseGate* gatePtr) override;
};

class Warn : public State {
public:
    Warn() { this->state = StateMachineState::WARNING; }
    std::unique_ptr<State> on_event(Event e) override;
    void on_update(BaseGate* gatePtr) override;
};
}; // namespace tod_safety_monitoring
