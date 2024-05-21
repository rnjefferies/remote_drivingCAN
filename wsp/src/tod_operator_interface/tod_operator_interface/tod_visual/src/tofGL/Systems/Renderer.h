// Copyright 2021 TUMFTM
#pragma once
#include "Core/RenderCommand.h"
#include "Scene/Components.h"
#include "ShaderSystem.h"
#include "entt/entt.hpp"
#include "Events/MouseEvent.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include <iostream>
#include <mutex>
#include <unistd.h>
#include <utility>

class Renderer {
public:
    // on startup
    static void GenerateMeshes(RenderableElementComponent &renderable);
    static void GenerateFrameBuffer(FrameBufferComponent &framebuffer, RenderableElementComponent &renderable);
    static void GenerateTextures(RenderableElementComponent& renderable);
    static void GenerateTexture(Texture &texture, const void* data, const int ShaderProgram, const int uniformId);
    static void CreateBuffer(Buffer &buffer, void* data, uint32_t size);

    // on update
    static void UpdateMeshes(const RenderableElementComponent &renderable, DynamicDataComponent &dynamic);
    static void SetupRenderingForFramebuffer(const FrameBufferComponent &framebuffer);
    static void RenderMeshes(const RenderableElementComponent &renderable);
    static void UpdateTexture(
        const Texture &tex, const unsigned int texIdx, const Buffer &buf, const unsigned int xOffset,
        const unsigned int yOffset, const unsigned width, const unsigned height, void *data);

    // debug
    static void PrintGLError(const int atIndex, const unsigned int objectName = 9999);
};
