// Copyright 2020 TUMFTM based on Cherno's Hazel
#include "Core/GraphicsContext.h"
#include "OpenGL/OpenGLContext.h"
#include "Window/GLFWWindow.h"

std::unique_ptr<GraphicsContext> GraphicsContext::create(void *window) {
    return std::make_unique<OpenGLContext>(static_cast<GLFWwindow *>(window));
}
