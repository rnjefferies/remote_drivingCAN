// Copyright 2020 TUMFTM based on Cherno's Hazel
#pragma once

#include "Events/Event.h"
#include "Events/MouseCodes.h"
#include <string>
#include <sstream>

class MouseScrolledEvent : public Event {
public:
    MouseScrolledEvent(float xOffset, float yOffset)
        : _xOffset(xOffset), _yOffset(yOffset) {}

    float GetXOffset() const { return _xOffset; }
    float GetYOffset() const { return _yOffset; }

    std::string ToString() const override {
        std::stringstream ss;
        ss << "MouseScrolledEvent: " << GetXOffset() << ", " << GetYOffset();
        return ss.str();
    }

    static EventType GetStaticType() { return EventType::MouseScrolled; }
    EventType GetEventType() const override { return GetStaticType(); }
    const char *GetName() const override { return "MouseScrolled"; }
    int GetCategoryFlags() const override { return (EventCategoryMouse | EventCategoryInput); }

private:
    float _xOffset, _yOffset;
};

class MouseMovedEvent : public Event {
    public:
    MouseMovedEvent(float xPosInPixel, float yPosInPixel, unsigned int windowHeight)
        : xPosInPixel(xPosInPixel), yPosInPixel(yPosInPixel), windowHeight(windowHeight) {}
    float xPosInPixel;
    float yPosInPixel;
    unsigned int windowHeight;

    static EventType GetStaticType() { return EventType::MouseMoved; }
    EventType GetEventType() const override { return GetStaticType(); }
    const char *GetName() const override { return "MouseMoved"; }
    int GetCategoryFlags() const override { return (EventCategoryMouse | EventCategoryInput); }
};

class MouseButtonEvent : public Event {
public:
    inline MouseCode GetMouseButton() const { return _button; }

    int GetCategoryFlags() const override { return (EventCategoryMouse | EventCategoryInput); }
protected:
    explicit MouseButtonEvent(MouseCode button)
        : _button(button) {}

    MouseCode _button;
};

class MouseButtonPressedEvent : public MouseButtonEvent {
public:
    explicit MouseButtonPressedEvent(MouseCode button)
        : MouseButtonEvent(button) {}

    std::string ToString() const override {
        std::stringstream ss;
        ss << "MouseButtonPressedEvent: " << _button;
        return ss.str();
    }

    static EventType GetStaticType() { return EventType::MouseButtonPressed; }
    EventType GetEventType() const override { return GetStaticType(); }
    const char *GetName() const override { return "MouseButtonPressed"; }
};

class MouseButtonReleasedEvent : public MouseButtonEvent {
public:
    explicit MouseButtonReleasedEvent(MouseCode button)
        : MouseButtonEvent(button) {}

    std::string ToString() const override {
        std::stringstream ss;
        ss << "MouseButtonReleasedEvent: " << _button;
        return ss.str();
    }

    static EventType GetStaticType() { return EventType::MouseButtonReleased; }
    EventType GetEventType() const override { return GetStaticType(); }
    const char *GetName() const override { return "MouseButtonReleased"; }
};
