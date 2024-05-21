// Copyright 2021 TUMFTM
#include "Systems/Renderer.h"

//std::unique_ptr<Renderer::SceneData> Renderer::s_SceneData = std::make_unique<Renderer::SceneData>();

void Renderer::GenerateFrameBuffer(FrameBufferComponent &framebuffer, RenderableElementComponent& renderable) {
    RenderCommand::ForFramebuffer::GenerateAndBind(&framebuffer.fbo);
    RenderCommand::ForRenderbuffer::Create(&framebuffer.rbo, GL_DEPTH24_STENCIL8, framebuffer.RenderWidth,
                                           framebuffer.RenderHeight);
    RenderCommand::ForFramebuffer::AttachTexture(
        GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderable.Meshes.front().Textures.front().Id);
    RenderCommand::ForFramebuffer::AttachRenderbuffer(framebuffer.rbo, GL_DEPTH_STENCIL_ATTACHMENT);
    RenderCommand::ForFramebuffer::CheckAndUnbind();
}

void Renderer::GenerateMeshes(RenderableElementComponent &renderable) {
    for (Mesh& mesh : renderable.Meshes) {
        if (mesh.Vertices.empty()) {
            ROS_ERROR("%s: Trying to generate mesh with empty vertices - filling up",
                      ros::this_node::getName().c_str());
            mesh.Vertices = Mesh::VectorOfThreeZeroVertices();
        }
        RenderCommand::ForVertexArray::GenerateAndBind(mesh.VAO);
        RenderCommand::ForBuffer::GenerateBindAndUpload(
            mesh.VertexBuffer, mesh.Vertices.data(), (uint32_t) mesh.Vertices.size() * sizeof(Vertex));
        if (!mesh.Indices.empty()) {
            RenderCommand::ForBuffer::GenerateBindAndUpload(
                mesh.IndexBuffer, mesh.Indices.data(), (uint32_t) mesh.Indices.size() * sizeof(mesh.Indices.front()));
        }
        RenderCommand::ForVertexAttributePtr::SetAndEnable(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) 0);
        RenderCommand::ForVertexAttributePtr::SetAndEnable(
            1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(sizeof(Vertex::Position)));
        RenderCommand::ForVertexAttributePtr::SetAndEnable(
            2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(sizeof(Vertex::Position) + sizeof(Vertex::TexCoord)));

        // unbind - order matters (VAO first)
        RenderCommand::ForVertexArray::Unbind(mesh.VAO);
        RenderCommand::ForBuffer::Unbind(mesh.IndexBuffer);
        RenderCommand::ForBuffer::Unbind(mesh.VertexBuffer);
    }
}

void Renderer::GenerateTexture(Texture &texture, const void* data, const int ShaderProgram, const int uniformId) {
    RenderCommand::ForTexture::GenerateActiveAndBind(texture, 0);
    RenderCommand::ForTexture::BindImage(texture, data);
    RenderCommand::ForTexture::SetParameter(texture, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    RenderCommand::ForTexture::SetParameter(texture, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    RenderCommand::ForTexture::SetParameter(texture, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    RenderCommand::ForTexture::SetParameter(texture, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    RenderCommand::ForTexture::Unbind(texture);
    ShaderSystem::setShaderProgramInt(ShaderProgram, texture.Name, uniformId);
}

void Renderer::CreateBuffer(Buffer &buffer, void* data, uint32_t size) {
    RenderCommand::ForBuffer::GenerateBindAndUpload(buffer, data, size);
    RenderCommand::ForBuffer::Unbind(buffer);
}

void Renderer::RenderMeshes(const RenderableElementComponent &renderable) {
    if (!renderable.DynamicShow || !renderable.StaticShow)
        return;
    ShaderSystem::useShaderProgram(renderable.ShaderProgram);
    RenderCommand::SetLineWidth(renderable.LineWidth);
    RenderCommand::SetPointSize(renderable.PointSize);
    for (const Mesh &mesh : renderable.Meshes) {
        if (!mesh.VAO.Id) {
            ROS_ERROR("%s: in renderMesh() no VAO to bind - skipping mesh",
                      ros::this_node::getName().c_str());
            continue;
        }
        if (mesh.Vertices.empty())
            continue;
        for (int i=0; i < mesh.Textures.size(); ++i)
            RenderCommand::ForTexture::ActiveAndBind(mesh.Textures.at(i), i);
        RenderCommand::ForVertexArray::Bind(mesh.VAO);
        if (!mesh.Indices.empty()) {
            RenderCommand::Draw::Elements(renderable.RenderMode, (uint32_t) mesh.Indices.size(), GL_UNSIGNED_INT, 0);
        } else {
            RenderCommand::Draw::Arrays(renderable.RenderMode, 0, (uint32_t) mesh.Vertices.size());
        }
        RenderCommand::ForVertexArray::Unbind(mesh.VAO);
    }
    ShaderSystem::useShaderProgram(0);
}

void Renderer::UpdateTexture(
    const Texture& tex, const unsigned int texIdx, const Buffer& buf, const unsigned int xOffset,
    const unsigned int yOffset, const unsigned width, const unsigned height, void* data) {
    RenderCommand::Enable(GL_TEXTURE_RECTANGLE);
    RenderCommand::ForBuffer::BindAndUpload(buf, data, width*height);
    RenderCommand::ForTexture::ActiveAndBind(tex, texIdx);
    RenderCommand::ForTexture::BindSubImage(tex, xOffset, yOffset, width, height, 0);
}

void Renderer::GenerateTextures(RenderableElementComponent& renderable) {
    for (Mesh &mesh : renderable.Meshes) {
        for (int i = 0; i < mesh.Textures.size(); ++i) {
            if (renderable.Meshes.front().Textures.at(i).Id == 0) {
                GenerateTexture(renderable.Meshes.front().Textures.at(i), 0, renderable.ShaderProgram, i);
            }
        }
    }
}

void Renderer::UpdateMeshes(const RenderableElementComponent &renderable,
                            DynamicDataComponent &dynamic) {
    std::lock_guard<std::mutex> lock(*dynamic.Mutex);
    if (!dynamic.HasNewData)
        return;

    for (const Mesh &mesh : renderable.Meshes) {
        if (!mesh.VAO.Id) {
            ROS_ERROR("%s: in uploadBufferDataToGPU() no VAO to bind - skipping mesh",
                      ros::this_node::getName().c_str());
            continue;
        }
        if (mesh.Vertices.empty())
            continue;
        for (uint32_t i=0; i < mesh.Textures.size(); ++i)
            RenderCommand::ForTexture::ActiveAndBind(mesh.Textures.at(i), i);
        RenderCommand::ForVertexArray::Bind(mesh.VAO);
        RenderCommand::ForBuffer::BindAndUpload(mesh.VertexBuffer, mesh.Vertices.data(),
                                                (uint32_t) mesh.Vertices.size() * sizeof(mesh.Vertices.front()));
        if (!mesh.Indices.empty()) {
            if (!mesh.IndexBuffer.Id)
                ROS_WARN("%s: in uploadBufferToGPU() - no EBO to bind but indices stored",
                         ros::this_node::getName().c_str());
            RenderCommand::ForBuffer::BindAndUpload(mesh.IndexBuffer, mesh.Indices.data(),
                                                    (uint32_t) mesh.Indices.size() * sizeof(mesh.Indices.front()));
        }

        // unbind - order matters (VAO first)
        RenderCommand::ForVertexArray::Unbind(mesh.VAO);
        RenderCommand::ForBuffer::Unbind(mesh.IndexBuffer);
        RenderCommand::ForBuffer::Unbind(mesh.VertexBuffer);
    }
    dynamic.HasNewData = false;
}

void Renderer::SetupRenderingForFramebuffer(const FrameBufferComponent &framebuffer) {
    RenderCommand::ForFramebuffer::Bind(framebuffer.fbo);
    RenderCommand::Enable(GL_DEPTH_TEST);
    RenderCommand::Enable(GL_MULTISAMPLE);
    RenderCommand::SetViewport(0, 0, framebuffer.RenderWidth, framebuffer.RenderHeight);
    RenderCommand::Clear();
}

void Renderer::PrintGLError(const int atIndex, const unsigned int objectName) {
    GLenum errorCode;
    while ((errorCode = glGetError()) != GL_NO_ERROR) {
        std::string error;
        switch (errorCode) {
        case GL_INVALID_ENUM:                  error = "INVALID_ENUM"; break;
        case GL_INVALID_VALUE:                 error = "INVALID_VALUE"; break;
        case GL_INVALID_OPERATION:             error = "INVALID_OPERATION"; break;
        case GL_STACK_OVERFLOW:                error = "STACK_OVERFLOW"; break;
        case GL_STACK_UNDERFLOW:               error = "STACK_UNDERFLOW"; break;
        case GL_OUT_OF_MEMORY:                 error = "OUT_OF_MEMORY"; break;
        case GL_INVALID_FRAMEBUFFER_OPERATION: error = "INVALID_FRAMEBUFFER_OPERATION"; break;
        }
        std::cout << error << std::endl;
    }
    std::cout << "at index: " << atIndex << " with object name " << objectName <<"\n";
}
