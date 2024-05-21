// Copyright 2020 TUMFTM
#include "Application.h"
#include "GLFW/glfw3.h"

Application* Application::_appInstance = nullptr;

Application::Application(int argc, char** argv, const std::string &name) {
    _ros = std::make_unique<RosInterface>(argc, argv);
    _ros->init();
    _window = Window::Create(WindowProps(name));
    _window->SetEventCallback(std::bind(&Application::onEvent, this, std::placeholders::_1));
    _appInstance = this;
    RenderCommand::Init();
    if (_ros->debug)
        setDebugMode();
}

void Application::run() {
    RenderCommand::SetClearColor({ 0.1f, 0.1f, 0.1f, 1 });
    _activeScene->Init(_window->GetWidth(), _window->GetHeight());
    ros::Rate r(144);
    while (ros::ok() && _isRunning) {
        float time_now = (float)glfwGetTime();
        _deltaTime = time_now - _lastFrameTime;
        _lastFrameTime = time_now;
        _activeScene->OnUpdate();
        _window->OnUpdate();
        r.sleep();
    }
}

void Application::close() {
    _isRunning = false;
}

void Application::setDebugMode() {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();
}

void Application::onEvent(Event& e) {
    EventDispatcher dispatcher(e);
    dispatcher.Dispatch<MouseMovedEvent>([this](auto&&... args){
        return this->handleMouseMovedEvent(std::forward<decltype(args)>(args)...);
    });
    dispatcher.Dispatch<MouseButtonPressedEvent>([this](auto&&... args){
        return this->handleMouseButtonPressedEvent(std::forward<decltype(args)>(args)...);
    });
    dispatcher.Dispatch<KeyPressedEvent>([this](auto&&... args){
        return this->handleKeyPressedEvent(std::forward<decltype(args)>(args)...);
    });
    dispatcher.Dispatch<WindowResizeEvent>([this](auto&&... args){
        return this->handleWindowResizeEvent(std::forward<decltype(args)>(args)...);
    });

    auto view = _activeScene->_registry.view<CameraComponent>();
    for (auto entity : view) {
        auto &camera = _activeScene->_registry.get<CameraComponent>(entity);
        if (camera.Controllable) {
            _camController.onEvent(e, camera, _deltaTime);
        }
    }
}
void Application::handleMouseMovedEvent(MouseMovedEvent& e) {
    _mousePosition = CursorPosition::getRealWorldCoordinates(e,
        _activeScene->_view, _activeScene->_projection);
}

void Application::handleMouseButtonPressedEvent(MouseButtonPressedEvent& e) {
    std::lock_guard<std::mutex> lock(_mousePositionMutex);
    _ros->setMouseClickForPublish(_mousePosition);
}

void Application::handleKeyPressedEvent(KeyPressedEvent& e) {
    tod_msgs::KeyPress msg;
    msg.key = static_cast<int>(e.GetKeyCode());
    _ros->setKeyPressForPublish(msg);
}

void Application::handleWindowResizeEvent(WindowResizeEvent& e) {
    auto view = _activeScene->_registry.view<FrameBufferComponent>();
    for (auto entity : view) {
        auto &framebuffer = _activeScene->_registry.get<FrameBufferComponent>(entity);
        if (framebuffer.IsDefaultFramebuffer) {
            framebuffer.RenderWidth = e.GetWidth();
            framebuffer.RenderHeight = e.GetHeight();
            auto& camera = _activeScene->_registry.get<CameraComponent>(framebuffer.CameraEntity);
            CameraSystem::OnWindowSizeChanged(camera, e.GetWidth(), e.GetHeight());
        }
    }
}
