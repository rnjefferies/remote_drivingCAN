// Copyright 2020 TUMFTM based on Cherno's Hazel
#pragma once
#include "Core/RendererAPI.h"

class OpenGLRendererAPI : public RendererAPI {
public:
    void Init() override;
    void Enable(const uint32_t capability) override;
    void SetBlendFunc(const uint32_t sfactor, const uint32_t dfactor) override;
    void SetViewport(const uint32_t x, const uint32_t y, const uint32_t width, const uint32_t height) override;

    void SetClearColor(const glm::vec4 &color) override;
    void Clear() override;
    void SetLineWidth(const float linewidth) override;
    void SetPointSize(const float pointsize) override;
    void SetPixelStorageMode(const uint32_t pname, const int32_t param) override;
    void DrawElements(const uint32_t mode, const uint32_t count, const uint32_t type, const void* data) override;
    void DrawArrays(const uint32_t mode, const uint32_t first, const uint32_t count) override;

    void GenerateBuffer(uint32_t *id) override;
    void GenerateVertexArray(uint32_t *id) override;
    void GenerateTexture(uint32_t *id) override;
    void GenerateFramebuffer(uint32_t *id) override;
    void BindFramebuffer(const uint32_t target, const uint32_t id) override;
    void GenerateRenderbuffer(uint32_t *id) override;
    void BindRenderbuffer(const uint32_t target, const uint32_t id) override;
    void AttachTextureToFramebuffer(const uint32_t target, const uint32_t attachment,
                                    const uint32_t textarget, const unsigned int texture, int level) override;
    void AllocateRenderBufferMemory(const uint32_t format, int samples, int width, int height) override;
    void AttachRenderbufferToFramebuffer(unsigned int renderbuffer, uint32_t attachment) override;
    void BindBuffer(const uint32_t target, const uint32_t id) override;
    void BindVertexArray(const uint32_t id) override;
    void BindTexture(const uint32_t type, const uint32_t id) override;
    void ActiveTexture(const uint32_t number) override;
    void BindImageToTexture(const uint32_t target, const int32_t level, const int32_t internalformat,
                            const uint32_t width, const uint32_t height, const int32_t border,
                            const uint32_t format, const uint32_t type, const void *data) override;
    void BindSubImageToTexture(const uint32_t target, const int32_t level, const int32_t xOffset,
                               const int32_t yOffset, const uint32_t width, const uint32_t height,
                               const uint32_t format, const uint32_t type, const void *data) override;
    void SetTextureParameteri(const uint32_t target, const uint32_t pname, const uint32_t param) override;
    void UploadToBuffer(const uint32_t target, const void* data, const uint32_t size, const uint32_t usage) override;
    void UnbindBuffer(const uint32_t target) override;
    void UnbindVertexArray() override;
    void UnbindRenderbuffer() override;
    void UnbindFramebuffer() override;
    void UnbindTexture(const uint32_t type) override;
    bool CheckFramebufferStatus() override;
    void SetVertexAttributePtr(const uint32_t index, const int32_t size, const uint32_t type,
                               const bool normalized, const uint32_t stride, const void *pointer) override;
    void EnableVertexAttributePtr(const uint32_t index) override;
};
