// Copyright 2020 TUMFTM based on Cherno's Hazel
#pragma once

#include "Events/Event.h"
#include <sstream>
#include <string>

class WindowCloseEvent : public Event {
public:
    WindowCloseEvent() = default;
    static EventType GetStaticType() { return EventType::WindowClose; }
    EventType GetEventType() const override { return GetStaticType(); }
    const char *GetName() const override { return "WindowClose"; }
    int GetCategoryFlags() const override { return (EventCategoryApplication); }

    std::string ToString() const override {
        std::stringstream ss;
        ss << "WindowCloseEvent";
        return ss.str();
    }
};

class WindowResizeEvent : public Event {
    public:
        WindowResizeEvent(unsigned int width, unsigned int height)
            : _width(width), _height(height) {}
        inline unsigned int GetWidth() const { return _width; }
        inline unsigned int GetHeight() const { return _height; }
        std::string ToString() const override {
            std::stringstream ss;
            ss << "WindowResizeEvent: " << _width << ", " << _height;
            return ss.str();
        }

    static EventType GetStaticType() { return EventType::WindowResize; }
    EventType GetEventType() const override { return GetStaticType(); }
    const char *GetName() const override { return "WindowResize"; }
    int GetCategoryFlags() const override { return (EventCategoryApplication); }
    private:
        unsigned int _width, _height;
};

