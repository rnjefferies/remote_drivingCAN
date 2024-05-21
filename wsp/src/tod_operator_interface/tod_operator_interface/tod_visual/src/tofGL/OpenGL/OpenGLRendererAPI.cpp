// Copyright 2020 TUMFTM based on Cherno's Hazel
#include "OpenGL/OpenGLRendererAPI.h"
#include <glad/glad.h>

void OpenGLRendererAPI::Init() {
    Enable(GL_BLEND);
    SetBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    Enable(GL_DEPTH_TEST);
}

void OpenGLRendererAPI::Enable(const uint32_t capability) {
    glEnable(capability);
}

void OpenGLRendererAPI::SetBlendFunc(const uint32_t sfactor, const uint32_t dfactor) {
    glBlendFunc(sfactor, dfactor);
}

void OpenGLRendererAPI::SetViewport(const uint32_t x, const uint32_t y,
                                    const uint32_t width, const uint32_t height) {
    glViewport(x, y, width, height);
}

void OpenGLRendererAPI::SetClearColor(const glm::vec4 &color) {
    glClearColor(color.r, color.g, color.b, color.a);
}

void OpenGLRendererAPI::Clear() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void OpenGLRendererAPI::SetLineWidth(const float linewidth) {
    glLineWidth(linewidth);
}

void OpenGLRendererAPI::SetPointSize(const float pointsize) {
    glPointSize(pointsize);
}

void OpenGLRendererAPI::SetPixelStorageMode(const uint32_t pname, const int32_t param) {
    glPixelStorei(pname, param);
}

void OpenGLRendererAPI::DrawElements(const uint32_t mode, const uint32_t count,
                                     const uint32_t type, const void* data) {
    glDrawElements(mode, count, type, data);
}

void OpenGLRendererAPI::DrawArrays(const uint32_t mode, const uint32_t first, const uint32_t count) {
    glDrawArrays(mode, first, count);
}

void OpenGLRendererAPI::GenerateBuffer(uint32_t *id) {
    glGenBuffers(1, id);
}

void OpenGLRendererAPI::GenerateVertexArray(uint32_t *id) {
    glGenVertexArrays(1, id);
}

void OpenGLRendererAPI::GenerateTexture(uint32_t *id) {
    glGenTextures(1, id);
}

void OpenGLRendererAPI::GenerateFramebuffer(uint32_t *id) {
    glGenFramebuffers(1, id);
}

void OpenGLRendererAPI::BindRenderbuffer(const uint32_t target, const uint32_t id) {
    glBindRenderbuffer(target, id);
}

void OpenGLRendererAPI::GenerateRenderbuffer(uint32_t *id) {
    glGenRenderbuffers(1, id);
}

void OpenGLRendererAPI::BindFramebuffer(const uint32_t target, const uint32_t id) {
    glBindFramebuffer(target, id);
}
void OpenGLRendererAPI::BindBuffer(const uint32_t target, const uint32_t id) {
    glBindBuffer(target, id);
}

void OpenGLRendererAPI::AttachTextureToFramebuffer(const uint32_t target, const uint32_t attachment,
                                                   const uint32_t textarget, const unsigned int texture, int level) {
    glFramebufferTexture2D(target, attachment, textarget, texture, level);
}

void OpenGLRendererAPI::AllocateRenderBufferMemory(const uint32_t format, int samples, int width, int height) {
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, format, width, height);
}

void OpenGLRendererAPI::AttachRenderbufferToFramebuffer(unsigned int renderbuffer, uint32_t attachment) {
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, attachment, GL_RENDERBUFFER, renderbuffer);
}

void OpenGLRendererAPI::BindVertexArray(const uint32_t id) {
    glBindVertexArray(id);
}

void OpenGLRendererAPI::BindTexture(const uint32_t type, const uint32_t id) {
    glBindTexture(type, id);
}

void OpenGLRendererAPI::ActiveTexture(const uint32_t number) {
    glActiveTexture(GL_TEXTURE0 + number);
}

void OpenGLRendererAPI::BindImageToTexture(
    const uint32_t target, const int32_t level, const int32_t internalformat, const uint32_t width,
    const uint32_t height, const int32_t border, const uint32_t format, const uint32_t type,
    const void *data) {
    glTexImage2D(target, level, internalformat, width, height, border, format, type, data);
}

void OpenGLRendererAPI::BindSubImageToTexture(
    const uint32_t target, const int32_t level, const int32_t xOffset, const int32_t yOffset,
    const uint32_t width, const uint32_t height, const uint32_t format, const uint32_t type,
    const void *data) {
    glTexSubImage2D(target, level, xOffset, yOffset, width, height, format, type, data);
}

void OpenGLRendererAPI::SetTextureParameteri(const uint32_t target, const uint32_t pname, const uint32_t param) {
    glTexParameteri(target, pname, param);
}

void OpenGLRendererAPI::UploadToBuffer(const uint32_t target, const void *data,
                                       const uint32_t size, const uint32_t usage) {
    glBufferData(target, size, data, usage);
}

void OpenGLRendererAPI::UnbindBuffer(const uint32_t target) {
    glBindBuffer(target, 0);
}

void OpenGLRendererAPI::UnbindVertexArray() {
    glBindVertexArray(0);
}

void OpenGLRendererAPI::UnbindRenderbuffer() {
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
}

void OpenGLRendererAPI::UnbindFramebuffer() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void OpenGLRendererAPI::UnbindTexture(const uint32_t type) {
    glBindTexture(type, 0);
}

bool OpenGLRendererAPI::CheckFramebufferStatus() {
    return glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE;
}

void OpenGLRendererAPI::SetVertexAttributePtr(
    const uint32_t index, const int32_t size, const uint32_t type, const bool normalized,
    const uint32_t stride, const void* pointer) {
    glVertexAttribPointer(index, size, type, normalized, stride, pointer);
}

void OpenGLRendererAPI::EnableVertexAttributePtr(const uint32_t index) {
    glEnableVertexAttribArray(index);
}
