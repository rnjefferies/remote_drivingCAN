// Copyright 2020 TUMFTM based on Cherno's Hazel
#include "Core/Window.h"
#include "Window/GLFWWindow.h"

std::unique_ptr<Window> Window::Create(const WindowProps& props) {
    return std::make_unique<GLFWWindow>(props);
}
