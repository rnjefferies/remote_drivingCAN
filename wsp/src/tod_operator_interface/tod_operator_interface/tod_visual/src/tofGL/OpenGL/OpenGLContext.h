// Copyright 2020 TUMFTM based on Cherno's Hazel
#pragma once
#include "Core/GraphicsContext.h"

struct GLFWwindow;

class OpenGLContext : public GraphicsContext {
public:
    explicit OpenGLContext(GLFWwindow *windowHandle);
    void init() override;
    void swapBuffers() override;

private:
    GLFWwindow *m_WindowHandle;
};
