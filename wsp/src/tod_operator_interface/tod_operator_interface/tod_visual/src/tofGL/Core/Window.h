// Copyright 2020 TUMFTM based on Cherno's Hazel
#pragma once
#include <memory>
#include <functional>
#include "Events/Event.h"
#include <string>

struct WindowProps {
    std::string Title;
    uint32_t Width;
    uint32_t Height;

    WindowProps(const std::string &title = "tod_visual", uint32_t width = 1280,
        uint32_t height = 720) : Title(title), Width(width), Height(height) { }
};

class Window {
public:
    using EventCallbackFn = std::function<void(Event&)>;
    virtual ~Window() = default;
    virtual void OnUpdate() = 0;
    virtual uint32_t GetWidth() const = 0;
    virtual uint32_t GetHeight() const = 0;
    virtual void SetEventCallback(const EventCallbackFn& callback) = 0;
    virtual void *GetNativeWindow() const = 0;

    static std::unique_ptr<Window> Create(const WindowProps &props = WindowProps());
};
