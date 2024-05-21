// Copyright 2020 TUMFTM
#pragma once
#include "Core/Window.h"
#include "Core/RenderCommand.h"
#include "Core/CameraController.h"
#include "Core/CursorPosition.h"
#include "Core/RosInterface.h"
#include "Scene/Scene.h"
#include "Scene/Components.h"
#include "Events/MouseEvent.h"
#include <functional>
#include <string>
#include <mutex>
#include <memory>
#include <utility>


class Application {
public:
    Application(int argc, char** argv, const std::string &name = "tod_visual");
    virtual ~Application() = default;

    void onEvent(Event& e);
    Window &getWindow() { return *_window; }
    void run();
    void close();
    static Application &get() { return *_appInstance; }
    std::mutex _mousePositionMutex;
    geometry_msgs::Point _mousePosition;

protected:
    std::shared_ptr<Scene> _activeScene;
    std::unique_ptr<RosInterface> _ros;

private:
    CameraController _camController;
    std::unique_ptr<Window> _window;
    bool _isRunning = true;
    float _lastFrameTime = 0.0f;
    float _deltaTime = 0.0f;
    static Application *_appInstance;
    void setDebugMode();
    void handleMouseMovedEvent(MouseMovedEvent& e);
    void handleMouseButtonPressedEvent(MouseButtonPressedEvent& e);
    void handleKeyPressedEvent(KeyPressedEvent& e);
    void handleWindowResizeEvent(WindowResizeEvent& e);
};
