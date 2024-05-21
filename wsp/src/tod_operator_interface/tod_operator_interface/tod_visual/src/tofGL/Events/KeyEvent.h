// Copyright 2020 TUMFTM based on Cherno's Hazel
#pragma once

#include "Events/Event.h"
#include "Events/KeyCodes.h"
#include "sstream"
#include <string>

class KeyEvent : public Event {
public:
    KeyCode GetKeyCode() const { return _keyCode; }
    int GetCategoryFlags() const override { return (EventCategoryKeyboard | EventCategoryInput); }
protected:
    explicit KeyEvent(KeyCode keycode)
        : _keyCode(keycode) {}

    KeyCode _keyCode;
};

class KeyPressedEvent : public KeyEvent {
public:
    explicit KeyPressedEvent(KeyCode keycode, int repeatCount)
        : KeyEvent(keycode), _repeatCount(repeatCount) {}

    int GetRepeatCount() const { return _repeatCount; }
    std::string ToString() const override {
        std::stringstream ss;
        ss << "KeyPressedEvent: " << _keyCode << " (" << _repeatCount << " repeats)";
        return ss.str();
    }

    static EventType GetStaticType() { return EventType::KeyPressed; }
    EventType GetEventType() const override { return GetStaticType(); }
    const char *GetName() const override { return "KeyPressed"; }
private:
    int _repeatCount;
};

class KeyReleasedEvent : public KeyEvent {
public:
    explicit KeyReleasedEvent(KeyCode keycode)
        : KeyEvent(keycode) {}

    std::string ToString() const override {
        std::stringstream ss;
        ss << "KeyReleasedEvent: " << _keyCode;
        return ss.str();
    }
    static EventType GetStaticType() { return EventType::KeyReleased; }
    EventType GetEventType() const override { return GetStaticType(); }
    const char *GetName() const override { return "KeyReleased"; }
};
