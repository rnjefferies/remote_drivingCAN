// Copyright 2021 Simon Hoffmann
#pragma once
#include "ros/ros.h"
#include "ros/debug.h"
#include <vector>
#include <map>
#include <thread>
#include <iostream>
#include "tod_safety_monitoring/GateStateMachine.h"
#include "tod_safety_monitoring/Utils.h"

namespace tod_safety_monitoring{
class BaseGate {
private:
    std::unique_ptr<State> _currentState = std::make_unique<Startup>();
    bool _verbose{false};

public:
    BaseGate() = default;
    BaseGate(const BaseGate&) = default;
    BaseGate(BaseGate&&) = default;
    BaseGate& operator=(const BaseGate&) = default;
    BaseGate& operator=(BaseGate&&) = default;
    virtual ~BaseGate() = default;

    void handle_gate_event(Event e) {
        auto new_state = _currentState->on_event(e);
        if (new_state) {
            if (_verbose) {
                std::cerr << "\033[1;34m" << "[SAFTY GATE - " << ros::this_node::getName() << "]: "
                    << _currentState->state << " --[" << e << "]--> " << new_state->state << "\033[0;49m"<< std::endl;
            }
            _currentState = std::move(new_state);
        }
    }
    void set_verbosity(const bool verbose) {_verbose = verbose;}
    virtual void open_on_update() = 0;
    virtual void startup_on_update() = 0;
    virtual void initialized_on_update() = 0;
    virtual void closed_on_update() = 0;
    virtual void warn_on_update() = 0;
    StateMachineState get_current_state() {
        return _currentState->state;
    }

protected:
    void update(){
        _currentState->on_update(this);
    }
};
}; // namespace tod_safety_monitoring
