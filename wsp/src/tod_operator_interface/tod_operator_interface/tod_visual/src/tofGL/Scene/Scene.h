// Copyright 2021 TUMFTM
#pragma once
#include <entt/entt.hpp>
#include <glad/glad.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/string_cast.hpp>
#include <Systems/VRSystem.h>
#include <string>

class Entity;

class Scene {
public:
    Scene();
    ~Scene() {}

    Entity CreateEntity(const std::string& name);
    void Init(const unsigned int width, const unsigned int height);
    void RenderMesh();
    void OnUpdate();
    void UpdateModel();
    void UpdateCameras();
    void UploadData();
    int RenderOnFramebuffer();
    void UpdateProjectionAndView();
    void setBaseFootPrint(const entt::entity &entitiy);

    entt::registry _registry;
    glm::mat4 _view;
    glm::mat4 _projection;

private:
    friend class Entity;
    entt::entity _baseFootPrint{entt::null};
    VRSystem _vrSystem;
    bool _vrMode{false};

    bool modelPoseChanged();
};
