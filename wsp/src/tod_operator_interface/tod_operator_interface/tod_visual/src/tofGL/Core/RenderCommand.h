// Copyright 2021 TUMFTM
#pragma once
#include "Core/DataContainer.h"
#include "Core/RendererAPI.h"
#include <memory>
class RenderCommand {
public:
    static void Init() {
        s_RendererAPI->Init();
    }

    static void Enable(const uint32_t capability) {
        s_RendererAPI->Enable(capability);
    }

    static void SetViewport(const uint32_t x, const uint32_t y,
                            const uint32_t width, const uint32_t height) {
        s_RendererAPI->SetViewport(x, y, width, height);
    }

    static void SetClearColor(const glm::vec4 &color) {
        s_RendererAPI->SetClearColor(color);
    }

    static void Clear() {
        s_RendererAPI->Clear();
    }

    static void SetLineWidth(const float linewidth) {
        s_RendererAPI->SetLineWidth(linewidth);
    }

    static void SetPointSize(const float pointsize) {
        s_RendererAPI->SetPointSize(pointsize);
    }

    static void SetPixelStorageMode(const uint32_t pname, const int32_t param) {
        s_RendererAPI->SetPixelStorageMode(pname, param);
    }

    class Draw {
    public:
        static void Elements(const uint32_t mode, const uint32_t count, const uint32_t type, const void* data) {
            s_RendererAPI->DrawElements(mode, count, type, data);
        }

        static void Arrays(const uint32_t mode, const uint32_t first, const uint32_t count) {
            s_RendererAPI->DrawArrays(mode, first, count);
        }
    };

    class ForBuffer {
    public:
        static void Generate(Buffer &buffer) {
            s_RendererAPI->GenerateBuffer(&buffer.Id);
        }

        static void Bind(const Buffer &buffer) {
            s_RendererAPI->BindBuffer(buffer.Target, buffer.Id);
        }

        static void Upload(const Buffer &buffer, const void* data, const uint32_t size) {
            s_RendererAPI->UploadToBuffer(buffer.Target, data, size, buffer.Usage);
        }

        static void Unbind(const Buffer &buffer) {
            s_RendererAPI->UnbindBuffer(buffer.Target);
        }

        static void GenerateAndBind(Buffer &buffer) {
            Generate(buffer);
            Bind(buffer);
        }

        static void BindAndUpload(const Buffer &buffer, const void* data, const uint32_t size) {
            Bind(buffer);
            Upload(buffer, data, size);
        }

        static void GenerateBindAndUpload(Buffer &buffer, const void* data, const uint32_t size) {
            Generate(buffer);
            BindAndUpload(buffer, data, size);
        }
    };


    class ForTexture {
    public:
        static void Generate(Texture &texture) {
            s_RendererAPI->GenerateTexture(&texture.Id);
        }

        static void Bind(const Texture &texture) {
            s_RendererAPI->BindTexture(texture.Type, texture.Id);
        }

        static void Active(const uint32_t textureNumber) {
            s_RendererAPI->ActiveTexture(textureNumber);
        }

        static void Unbind(const Texture &texture) {
            s_RendererAPI->UnbindTexture(texture.Type);
        }

        static void BindImage(const Texture &texture, const void* data) {
            s_RendererAPI->BindImageToTexture(texture.Type, 0, texture.Format, texture.Width, texture.Height, 0,
                                         texture.Format, GL_UNSIGNED_BYTE, data);
        }

        static void BindSubImage(const Texture &texture, const int32_t xOffset, const int32_t yOffset,
                                 const uint32_t width, const uint32_t height, const void* data) {
            s_RendererAPI->BindSubImageToTexture(texture.Type, 0, xOffset, yOffset, width, height,
                                            texture.Format, GL_UNSIGNED_BYTE, data);
        }

        static void SetParameter(const Texture &texture, const uint32_t pname, const uint32_t param) {
            s_RendererAPI->SetTextureParameteri(texture.Type, pname, param);
        }

        static void ActiveAndBind(const Texture &texture, const uint32_t number) {
            Active(number);
            Bind(texture);
        }

        static void GenerateActiveAndBind(Texture &texture, const uint32_t number) {
            Generate(texture);
            ActiveAndBind(texture, number);
        }

        static void GenerateAndBind(Texture &texture, const uint32_t number) {
            Generate(texture);
            Bind(texture);
        }
    };


    class ForVertexArray {
    public:
        static void Generate(VertexArray &vao) {
            s_RendererAPI->GenerateVertexArray(&vao.Id);
        }

        static void Bind(const VertexArray &vao) {
            s_RendererAPI->BindVertexArray(vao.Id);
        }

        static void Unbind(const VertexArray &vao) {
            s_RendererAPI->UnbindVertexArray();
        }

        static void GenerateAndBind(VertexArray &vao) {
            Generate(vao);
            Bind(vao);
        }
    };


    class ForVertexAttributePtr {
    public:
        static void Set(const uint32_t index, const int32_t size, const uint32_t type,
                        const bool normalized, const uint32_t stride, const void* pointer) {
            s_RendererAPI->SetVertexAttributePtr(index, size, type, normalized, stride, pointer);
        }

        static void Enable(const uint32_t index) {
            s_RendererAPI->EnableVertexAttributePtr(index);
        }

        static void SetAndEnable(const uint32_t index, const int32_t size, const uint32_t type,
                                 const bool normalized, const uint32_t stride, const void* pointer) {
            Set(index, size, type, normalized, stride, pointer);
            Enable(index);
        }
    };


    class ForFramebuffer {
    public:
        static void Bind(const uint32_t id) {
            s_RendererAPI->BindFramebuffer(GL_FRAMEBUFFER, id);
        }

        static void Generate(uint32_t *fbo) {
            s_RendererAPI->GenerateFramebuffer(fbo);
        }


        static void GenerateAndBind(uint32_t *fbo) {
            Generate(fbo);
            Bind(*fbo);
        }

        static void Unbind() {
            s_RendererAPI->UnbindFramebuffer();
        }

        static void AttachTexture(const uint32_t target, const uint32_t attachment,
                                  const uint32_t textarget, const unsigned int texture) {
            s_RendererAPI->AttachTextureToFramebuffer(target, attachment, textarget, texture, 0);
        }

        static void AttachRenderbuffer(unsigned int renderbuffer, uint32_t attachement) {
            s_RendererAPI->AttachRenderbufferToFramebuffer(renderbuffer, attachement);
        }

        static void Check() {
            if (s_RendererAPI->CheckFramebufferStatus())
                std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
        }

        static void CheckAndUnbind() {
            Check();
            Unbind();
        }
    };


    class ForRenderbuffer {
    public:
        static void Generate(uint32_t *rbo) {
            s_RendererAPI->GenerateRenderbuffer(rbo);
        }

        static void Bind(const uint32_t id) {
            s_RendererAPI->BindRenderbuffer(GL_RENDERBUFFER, id);
        }

        static void AllocateMemory(const uint32_t format, int samples, int width, int height) {
            s_RendererAPI->AllocateRenderBufferMemory(format, samples, width, height);
        }

        static void AllocateMemory(const uint32_t format, int width, int height) {
            s_RendererAPI->AllocateRenderBufferMemory(format, 0, width, height);
        }

        static void Unbind() {
            s_RendererAPI->UnbindRenderbuffer();
        }

        static void Create(uint32_t *rbo, const uint32_t format, int samples, int width, int height) {
            Generate(rbo);
            Bind(*rbo);
            AllocateMemory(format, samples, width, height);
            Unbind();
        }

        static void Create(uint32_t *rbo, const uint32_t format, int width, int height) {
            Create(rbo, format, 0, width, height);
        }
    };

private:
    static std::unique_ptr<RendererAPI> s_RendererAPI;
};
