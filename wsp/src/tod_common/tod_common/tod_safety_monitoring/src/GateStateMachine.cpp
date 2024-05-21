// Copyright 2021 Simon Hoffmann
#include "tod_safety_monitoring/GateStateMachine.h"
#include "tod_safety_monitoring/SafetyGate.h"
#include "tod_safety_monitoring/BaseGate.h"
namespace tod_safety_monitoring {
std::unique_ptr<State> Startup::on_event(Event e) {
    if (e == Event::INITIALIZED) return std::make_unique<Initialized>();
    return nullptr;
}

std::unique_ptr<State> Initialized::on_event(Event e) {
    switch (e) {
        case Event::OPEN: return std::make_unique<Open>();
        case Event::WARN: return std::make_unique<Warn>();
    }
    return nullptr;
}

std::unique_ptr<State> Closed::on_event(Event e) {
    switch (e) {
        case Event::RESET: return std::make_unique<Startup>();
    }
    return nullptr;
}

std::unique_ptr<State> Open::on_event(Event e) {
    switch (e) {
        case Event::WARN: return std::make_unique<Warn>();
        case Event::CLOSE: return std::make_unique<Closed>();
        case Event::RESET: return std::make_unique<Startup>();
    }
    return nullptr;
}

std::unique_ptr<State> Warn::on_event(Event e) {
    switch (e) {
        case Event::OPEN: return std::make_unique<Open>();
        case Event::CLOSE: return std::make_unique<Closed>();
        case Event::RESET: return std::make_unique<Startup>();
    }
    return nullptr;
}

void Startup::on_update(BaseGate* gatePtr) {
    gatePtr->startup_on_update();
}

void Initialized::on_update(BaseGate* gatePtr) {
    gatePtr->initialized_on_update();
}

void Closed::on_update(BaseGate* gatePtr) {
    gatePtr->closed_on_update();
}

void Open::on_update(BaseGate* gatePtr) {
    gatePtr->open_on_update();
}

void Warn::on_update(BaseGate* gatePtr) {
    gatePtr->warn_on_update();
}
}; // namespace tod_safety_monitoring
