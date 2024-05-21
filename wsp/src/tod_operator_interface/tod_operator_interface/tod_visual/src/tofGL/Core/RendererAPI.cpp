// Copyright 2020 TUMFTM
#include "Core/RendererAPI.h"
#include "OpenGL/OpenGLRendererAPI.h"

std::unique_ptr<RendererAPI> RendererAPI::Create() {
    return std::make_unique<OpenGLRendererAPI>();
}
