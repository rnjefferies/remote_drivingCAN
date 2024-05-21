// Copyright 2020 TUMFTM based on Cherno's Hazel
#include "OpenGL/OpenGLContext.h"
#include "iostream"
#include <glad/glad.h>
#include <GLFW/glfw3.h>

OpenGLContext::OpenGLContext(GLFWwindow *windowHandle)
    : m_WindowHandle(windowHandle) {}

void OpenGLContext::init() {
    glfwMakeContextCurrent(m_WindowHandle);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
    }
}

void OpenGLContext::swapBuffers() {
    glfwSwapBuffers(m_WindowHandle);
}
