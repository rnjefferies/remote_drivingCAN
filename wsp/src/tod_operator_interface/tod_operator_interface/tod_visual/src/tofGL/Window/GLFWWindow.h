// Copyright 2020 TUMFTM based on Cherno's Hazel
#pragma once
#include "Core/Window.h"
#include "Core/GraphicsContext.h"
#include "iostream"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Events/KeyCodes.h"
#include "Events/MouseCodes.h"
#include <memory>
#include <string>

class GLFWWindow : public Window {
public:
    explicit GLFWWindow(const WindowProps &props);
    ~GLFWWindow() override;

    void OnUpdate() override;
    unsigned int GetWidth() const override { return m_Data.Width; }
    unsigned int GetHeight() const override { return m_Data.Height; }
    void SetEventCallback(const EventCallbackFn &callback) override { m_Data.EventCallback = callback; }
    void *GetNativeWindow() const override { return m_Window; }

private:
    GLFWwindow *m_Window;
    std::unique_ptr<GraphicsContext> m_Context;
    struct WindowData {
        std::string Title;
        unsigned int Width, Height;
        bool VSync;
        EventCallbackFn EventCallback;
    };
    WindowData m_Data;

    virtual void Init(const WindowProps &props);
    virtual void Shutdown();
};
