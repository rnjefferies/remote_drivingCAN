// Copyright 2021 TUMFTM
#include <Systems/Renderer.h>
#include <Systems/CameraSystem.h>
#include <Systems/TransformSystem.h>
#include "Scene.h"
#include "Entity.h"
#include "Components.h"

Scene::Scene() { _vrMode = _vrSystem.vrInit(); }

Entity Scene::CreateEntity(const std::string &name) {
    Entity entity = {_registry.create(), this};
    entity.AddComponent<TransformComponent>();
    auto &tag = entity.AddComponent<TagComponent>();
    tag.Tag = name.empty() ? "Entity" : name;
    return entity;
}

void Scene::OnUpdate() {
    auto t0 = std::chrono::high_resolution_clock::now();
    UpdateCameras();

    auto t1 = std::chrono::high_resolution_clock::now();
    UpdateModel();

    auto t2 = std::chrono::high_resolution_clock::now();
    UploadData();

    auto t3 = std::chrono::high_resolution_clock::now();
    int nofFramebuffersRendered = RenderOnFramebuffer();

    auto t4 = std::chrono::high_resolution_clock::now();
    _vrSystem.updateVrPose();

    bool calcAndPrintFPS{false};
    if (calcAndPrintFPS) {
        static int frameCounter{0};
        static ros::Time tRes = ros::Time::now();
        static std::chrono::duration<double> mean0{0.0}, mean1{0.0}, mean2{0.0}, mean3{0.0}, meanTotal{0.0};
        ++frameCounter;
        mean0 += t1-t0;
        mean1 += t2-t1;
        mean2 += t3-t2;
        mean3 += t4-t3;
        meanTotal += t4-t0;
        if (ros::Time::now() >= tRes + ros::Duration(1.0)) {
            std::cout << std::endl << std::endl;
            ROS_WARN("visual fps %d", frameCounter);
            ROS_WARN_STREAM("on average update cameras took " << (mean0.count()*1000) / frameCounter << " ms.");
            ROS_WARN_STREAM("on average update model took " << (mean1.count()*1000) / frameCounter << " ms.");
            ROS_WARN_STREAM("on average upload new data took " << (mean2.count()*1000) / frameCounter << " ms.");
            ROS_WARN_STREAM("on average rendering took " << (mean3.count()*1000) / frameCounter << " ms."
                                                         << " --- for "<< nofFramebuffersRendered  << " Iterations");
            ROS_WARN_STREAM("     on average TOTAL render took " << (meanTotal.count()*1000) / frameCounter << " ms.");
            mean0 = mean1 = mean2 = mean3 = meanTotal = std::chrono::duration<double>(0.0);
            tRes = ros::Time::now();
            frameCounter = 0;
        }
    }
}

void Scene::UpdateModel() {
    auto &dynamic = _registry.get<DynamicDataComponent>(_baseFootPrint);
    std::lock_guard<std::mutex> lock(*dynamic.Mutex);
    auto view = _registry.view<RenderableElementComponent>();
    for (auto &entity : view) {
        auto &renderable = _registry.get<RenderableElementComponent>(entity);
        auto &transform = _registry.get<TransformComponent>(entity);
        glm::mat4 model = TransformSystem::LocalToWorld(transform);
        ShaderSystem::setShaderProgramMat4(renderable.ShaderProgram, "Model", model);
    }
    dynamic.HasNewData = false;
}

void Scene::UpdateCameras() {
    // Update Camera of each Framebuffer
    auto view = _registry.view<FrameBufferComponent>();
    for (auto &entity : view) {
        auto& camera = _registry.get<CameraComponent>(_registry.get<FrameBufferComponent>(entity).CameraEntity);
        auto& transform = _registry.get<TransformComponent>(_registry.get<FrameBufferComponent>(entity).CameraEntity);
        if (_registry.has<VRComponent>(entity)) {
            auto& vr = _registry.get<VRComponent>(entity);
            _vrSystem.calcProjectionMatrix(camera, vr);
            _vrSystem.calcViewMatrix(camera, vr, transform);
        } else {

            CameraSystem::CalcViewMatrix(camera, transform);
        }
    }
}


void Scene::UploadData() {
    auto view = _registry.view<DynamicDataComponent, RenderableElementComponent>();
    for (auto &entity : view) {
        auto &renderable = _registry.get<RenderableElementComponent>(entity);
        auto &dynamic = _registry.get<DynamicDataComponent>(entity);
        Renderer::UpdateMeshes(renderable, dynamic);
    }
}

int Scene::RenderOnFramebuffer() {
    int nofFramebuffersRendered{0};
    for (auto &entity : _registry.view<FrameBufferComponent>()) {
        auto &framebuffer = _registry.get<FrameBufferComponent>(entity);
        Renderer::SetupRenderingForFramebuffer(framebuffer);
        // use projection and view of camera assigned to framebuffer
        if (_registry.has<CameraComponent>(framebuffer.CameraEntity)) {
            _view = _registry.get<CameraComponent>(framebuffer.CameraEntity).View;
            _projection = _registry.get<CameraComponent>(framebuffer.CameraEntity).Projection;
            UpdateProjectionAndView();
        } else {
            ROS_ERROR_STREAM("No Camera assigned to Framebuffer of Entity %s: " <<
                             _registry.get<TagComponent>(entity).Tag);
        }
        RenderMesh();
        if (_registry.has<VRComponent>(entity)) {
            auto& renderable = _registry.get<RenderableElementComponent>(entity);
            auto &vr = _registry.get<VRComponent>(entity);
            VRSystem::submitTexture(renderable, vr);
        }
        ++nofFramebuffersRendered;
    }
    return nofFramebuffersRendered;
}

void Scene::UpdateProjectionAndView() {
    auto view = _registry.view<RenderableElementComponent>();
    for (auto &entity : view) {
        auto &renderable = _registry.get<RenderableElementComponent>(entity);
        ShaderSystem::setShaderProgramMat4(renderable.ShaderProgram, "ProjectionView", _projection * _view);
    }
}

void Scene::RenderMesh() {
    auto view = _registry.view<RenderableElementComponent>();
    for (auto &entity : view) {
        auto &renderable = _registry.get<RenderableElementComponent>(entity);
        if (_registry.has<VideoComponent>(entity)) {
            auto &video = _registry.get<VideoComponent>(entity);
            auto &mesh = renderable.Meshes.front();
            if (video.ImageMsg) {
                std::lock_guard<std::mutex> lock(*video.Mutex);
                int step = 0;
                for (int i=0; i < 3; ++i) {
                    auto& pxBuf = video.PixelBuffers.at(i);
                    Renderer::UpdateTexture(mesh.Textures.at(i), i, pxBuf.Buf, 0, 0,
                                            pxBuf.Width, pxBuf.Height, (void*) &video.ImageMsg->data.at(step));
                    step += pxBuf.Width * pxBuf.Height;
                }
                video.ImageMsg.reset();
            }
            if (video.ProjectionMsg) {
                std::lock_guard<std::mutex> lock(*video.Mutex);
                auto& pxBuf = video.PixelBuffers.at(3);
                Renderer::UpdateTexture(mesh.Textures.at(3), 3, pxBuf.Buf, 0, 0,
                                        pxBuf.Width, pxBuf.Height, (void*) video.ProjectionMsg->data.data());
                video.ProjectionMsg.reset();
            }
        }

        if (_registry.has<ExpirableComponent>(entity))
            renderable.DynamicShow = !(_registry.get<ExpirableComponent>(entity).expired());

        if (_registry.has<DynamicDataComponent>(entity)) {
            std::lock_guard<std::mutex> lock(*_registry.get<DynamicDataComponent>(entity).Mutex);
            Renderer::RenderMeshes(renderable);
        } else {
            Renderer::RenderMeshes(renderable);
        }
    }
}

void Scene::Init(const unsigned int width, const unsigned int height) {
    if (_vrMode)
        _vrSystem.createVrEntities(this, _baseFootPrint);
    {
        auto view = _registry.view<RenderableElementComponent>();
        for (auto entity : view) {
            auto &renderable = _registry.get<RenderableElementComponent>(entity);
            if (_registry.has<DynamicDataComponent>(entity)) {
                std::lock_guard<std::mutex> lock(*_registry.get<DynamicDataComponent>(entity).Mutex);
                Renderer::GenerateMeshes(renderable);
                Renderer::GenerateTextures(renderable);
            } else {
                Renderer::GenerateMeshes(renderable);
                Renderer::GenerateTextures(renderable);
            }
        }
    }

    {
        bool mainFramebufferSpecified{false};
        auto view = _registry.view<FrameBufferComponent>();
        for (auto entity : view) {
            auto &framebuffer = _registry.get<FrameBufferComponent>(entity);
            if (framebuffer.IsDefaultFramebuffer) {
                framebuffer.RenderWidth = width;
                framebuffer.RenderHeight = height;
                mainFramebufferSpecified = true;
            }
            auto& camera = _registry.get<CameraComponent>(framebuffer.CameraEntity);
            CameraSystem::OnWindowSizeChanged(camera, framebuffer.RenderWidth, framebuffer.RenderHeight);


            if (!framebuffer.IsDefaultFramebuffer) {
                auto &renderable = _registry.get<RenderableElementComponent>(entity);
                Renderer::GenerateFrameBuffer(framebuffer, renderable);
            }
        }
        if (!mainFramebufferSpecified)
            ROS_ERROR_STREAM("No default Framebuffer specified!");
    }

    {
        auto view = _registry.view<RenderableElementComponent, VideoComponent>();
        for (auto &entity : view) {
            auto &pixelBuffer = _registry.get<VideoComponent>(entity);
            for (auto& pxBuf : pixelBuffer.PixelBuffers)
                Renderer::CreateBuffer(pxBuf.Buf, 0, pxBuf.Width * pxBuf.Height);
        }
    }

    UpdateModel();
}

void Scene::setBaseFootPrint(const entt::entity &entity) { _baseFootPrint = entity; }

bool Scene::modelPoseChanged() {
    if (_baseFootPrint == entt::null) {
        ROS_ERROR("does not have handle of base_footprint entity in render loop");
        return false;
    }
    return _registry.get<DynamicDataComponent>(_baseFootPrint).HasNewData;
}
