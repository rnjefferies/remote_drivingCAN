// Copyright 2021 TUMFTM
#pragma once
#include "openvr/openvr.h"
#include <string>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <entt/entt.hpp>

struct CameraComponent;
struct VRComponent;
struct TransformComponent;
struct RenderableElementComponent;
class Scene;
class Entity;

class VRSystem {
private:
    void handleVRError(vr::EVRInitError err);
    glm::mat4 _headPose;
    glm::mat4 convertSteamVRMatrixToGLM(const vr::HmdMatrix34_t &matPose);
    glm::mat4 convertSteamVRMatrixToGLM(const vr::HmdMatrix44_t &matPose);
public:
    VRSystem() = default;
    ~VRSystem();

    bool vrInit();
    bool initCompositor();
    vr::IVRSystem *_pHMD{NULL};
    void calcProjectionMatrix(CameraComponent& camera, VRComponent& vr);
    void calcViewMatrix(CameraComponent& camera, VRComponent& vr, TransformComponent& transform);
    void updateVrPose();
    void createVrEntities(Scene* activeScene, entt::entity parent);
    void createVrEntity(Scene* activeScene, Entity parent, vr::EVREye eye);
    static void submitTexture(RenderableElementComponent& renderable, VRComponent& vr);
};
