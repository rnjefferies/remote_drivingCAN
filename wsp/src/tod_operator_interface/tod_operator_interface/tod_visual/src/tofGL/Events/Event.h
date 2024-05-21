// Copyright 2020 TUMFTM based on Cherno's Hazel
#pragma once
#include <string>

#define BIT(x) (1 << x)

enum class EventType {
    None = 0,
    WindowClose,
    WindowResize,
    WindowFocus,
    WindowLostFocus,
    WindowMoved,
    KeyPressed,
    KeyReleased,
    KeyTyped,
    MouseButtonPressed,
    MouseButtonReleased,
    MouseMoved,
    MouseScrolled
};

enum EventCategory {
    None = 0,
    EventCategoryApplication = BIT(0),
    EventCategoryInput = BIT(1),
    EventCategoryKeyboard = BIT(2),
    EventCategoryMouse = BIT(3),
    EventCategoryMouseButton = BIT(4)
};

class Event {
public:
    virtual ~Event() = default;

    bool Handled = false;

    virtual EventType GetEventType() const = 0;
    virtual const char *GetName() const = 0;
    virtual int GetCategoryFlags() const = 0;
    virtual std::string ToString() const { return GetName(); }

    bool IsInCategory(EventCategory category) {
        return GetCategoryFlags() & category;
    }
};

class EventDispatcher {
public:
    explicit EventDispatcher(Event &event)
        : _event(event) {  }

    template <typename T, typename F>
    void Dispatch(const F &func) {
        if (_event.GetEventType() == T::GetStaticType()) {
            func(static_cast<T &>(_event));
        }
    }
private:
    Event &_event;
};
