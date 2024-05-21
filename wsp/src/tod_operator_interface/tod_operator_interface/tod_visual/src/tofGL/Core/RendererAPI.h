// Copyright 2020 TUMFTM
#pragma once
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <memory>

class RendererAPI {
public:
    virtual ~RendererAPI() = default;
    virtual void Init() = 0;
    virtual void Enable(const uint32_t capability) = 0;
    virtual void SetBlendFunc(const uint32_t sfactor, const uint32_t dfactor) = 0;
    virtual void SetViewport(const uint32_t x, const uint32_t y, const uint32_t width, const uint32_t height) = 0;
    virtual void SetClearColor(const glm::vec4 &color) = 0;
    virtual void Clear() = 0;
    virtual void SetLineWidth(const float linewidth) = 0;
    virtual void SetPointSize(const float pointsize) = 0;
    virtual void SetPixelStorageMode(const uint32_t pname, const int32_t param) = 0;
    virtual void DrawElements(const uint32_t mode, const uint32_t count, const uint32_t type, const void* data) = 0;
    virtual void DrawArrays(const uint32_t mode, const uint32_t first, const uint32_t count) = 0;
    virtual void GenerateBuffer(uint32_t* id) = 0;
    virtual void GenerateVertexArray(uint32_t* id) = 0;
    virtual void GenerateTexture(uint32_t* id) = 0;
    virtual void GenerateFramebuffer(uint32_t* id) = 0;
    virtual void GenerateRenderbuffer(uint32_t* id) = 0;
    virtual void BindBuffer(const uint32_t target, const uint32_t id) = 0;
    virtual void BindFramebuffer(const uint32_t target, const uint32_t id) = 0;
    virtual void BindRenderbuffer(const uint32_t target, const uint32_t id) = 0;
    virtual void AllocateRenderBufferMemory(const uint32_t format, int samples, int width, int height) = 0;
    virtual void AttachRenderbufferToFramebuffer(unsigned int renderbuffer, uint32_t attachment) = 0;
    virtual void AttachTextureToFramebuffer(const uint32_t target, const uint32_t attachment,
        const uint32_t textarget, const unsigned int texture, int level) = 0;
    virtual void BindVertexArray(const uint32_t id) = 0;
    virtual void BindTexture(const uint32_t type, const uint32_t id) = 0;
    virtual void ActiveTexture(const uint32_t number) = 0;
    virtual void BindImageToTexture(const uint32_t target, const int32_t level, const int32_t internalformat,
        const uint32_t width, const uint32_t height, const int32_t border,
        const uint32_t format, const uint32_t type, const void *data) = 0;
    virtual void BindSubImageToTexture(const uint32_t target, const int32_t level, const int32_t xOffset,
        const int32_t yOffset, const uint32_t width, const uint32_t height,
        const uint32_t format, const uint32_t type, const void *data) = 0;
    virtual void SetTextureParameteri(const uint32_t target, const uint32_t pname, const uint32_t param) = 0;
    virtual void UploadToBuffer(const uint32_t target, const void* data,
        const uint32_t size, const uint32_t usage) = 0;
    virtual void UnbindBuffer(const uint32_t target) = 0;
    virtual void UnbindRenderbuffer() = 0;
    virtual void UnbindFramebuffer() = 0;
    virtual void UnbindVertexArray() = 0;
    virtual void UnbindTexture(const uint32_t type) = 0;
    virtual bool CheckFramebufferStatus() = 0;
    virtual void SetVertexAttributePtr(const uint32_t index, const int32_t size, const uint32_t type,
        const bool normalized, const uint32_t stride, const void *pointer) = 0;
    virtual void EnableVertexAttributePtr(const uint32_t index) = 0;
    static std::unique_ptr<RendererAPI> Create();
};

