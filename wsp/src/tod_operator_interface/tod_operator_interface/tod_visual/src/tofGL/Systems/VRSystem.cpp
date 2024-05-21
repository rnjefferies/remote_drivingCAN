// Copyright 2021 TUMFTM
#include "VRSystem.h"
#include <ros/package.h>
#include <ros/ros.h>
#include "Scene/Components.h"
#include "Scene/Scene.h"
#include "ShaderSystem.h"
#include "Systems/TransformSystem.h"

VRSystem::~VRSystem() {
    if (_pHMD) {
        vr::VR_Shutdown();
        _pHMD = NULL;
    }
}

bool VRSystem::vrInit() {
    _pHMD = NULL;

    if (!vr::VR_IsHmdPresent()) {
        ROS_INFO("HMD is not connected");
        return false;
    }
    if (!vr::VR_IsRuntimeInstalled()) {
        ROS_ERROR("OpenVR Runtime is not installed");
        return false;
    }
    vr::EVRInitError err = vr::VRInitError_None;
    _pHMD = vr::VR_Init(&err, vr::VRApplication_Scene);
    if (err != vr::VRInitError_None) {
        ROS_ERROR("Check if SteamVR is started!");
        _pHMD = NULL;
        return false;
    }
    return initCompositor();
}

bool VRSystem::initCompositor() {
    if (!_pHMD)
        return false;
    vr::EVRInitError peError = vr::VRInitError_None;
    if (!vr::VRCompositor()) {
        printf("Compositor initialization failed. See log file for details\n");
        return false;
    }
    return true;
}

void VRSystem::updateVrPose() {
    static char devClassChar[vr::k_unMaxTrackedDeviceCount];
    static vr::TrackedDevicePose_t trackedDevicePose[vr::k_unMaxTrackedDeviceCount];
    static glm::mat4 devicePose[vr::k_unMaxTrackedDeviceCount];
    static int validPoseCount;
    static std::string poseClasses;

    if (!_pHMD)
        return;

    vr::VRCompositor()->WaitGetPoses(trackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0);
    validPoseCount = 0;
    poseClasses = "";
    for (int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice) {
        if (trackedDevicePose[nDevice].bPoseIsValid) {
            validPoseCount++;
            devicePose[nDevice] = convertSteamVRMatrixToGLM(trackedDevicePose[nDevice].mDeviceToAbsoluteTracking);
            if (devClassChar[nDevice] == 0) {
                switch (_pHMD->GetTrackedDeviceClass(nDevice)) {
                case vr::TrackedDeviceClass_Controller:        devClassChar[nDevice] = 'C'; break;
                case vr::TrackedDeviceClass_HMD:               devClassChar[nDevice] = 'H'; break;
                case vr::TrackedDeviceClass_Invalid:           devClassChar[nDevice] = 'I'; break;
                case vr::TrackedDeviceClass_GenericTracker:    devClassChar[nDevice] = 'G'; break;
                case vr::TrackedDeviceClass_TrackingReference: devClassChar[nDevice] = 'T'; break;
                default:                                       devClassChar[nDevice] = '?'; break;
                }
            }
            poseClasses += devClassChar[nDevice];
        }
    }
    if (trackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid) {
        _headPose = devicePose[vr::k_unTrackedDeviceIndex_Hmd];
        _headPose = glm::inverse(_headPose);
    }
}

void VRSystem::submitTexture(RenderableElementComponent& renderable, VRComponent& vr) {
    vr::Texture_t eyeTexture = {
        (void*)(uintptr_t)renderable.Meshes.at(0).Textures.at(0).Id,
        vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
    vr::VRCompositor()->Submit(vr.Eye, &eyeTexture);
}

void VRSystem::calcProjectionMatrix(CameraComponent& camera, VRComponent& vr) {
    if (!_pHMD) {
        camera.Projection = glm::mat4(1.0f);
    } else {
        vr::HmdMatrix44_t mat = _pHMD->GetProjectionMatrix(vr.Eye, camera.NearPlane , camera.FarPlane);
        camera.Projection = convertSteamVRMatrixToGLM(mat);
    }
}

void VRSystem::calcViewMatrix(CameraComponent& camera, VRComponent& vr, TransformComponent& transform) {
    if (!_pHMD) {
        camera.View = glm::mat4(1.0f);
    } else {
        vr::HmdMatrix34_t eyeToHeadTransform = _pHMD->GetEyeToHeadTransform(vr.Eye);
        camera.View =  convertSteamVRMatrixToGLM(eyeToHeadTransform) *_headPose
            * glm::inverse(TransformSystem::LocalToWorld(transform));
    }
}

void VRSystem::handleVRError(vr::EVRInitError err) {
    throw std::runtime_error(vr::VR_GetVRInitErrorAsEnglishDescription(err));
}

glm::mat4 VRSystem::convertSteamVRMatrixToGLM(const vr::HmdMatrix44_t &matPose) {
    glm::mat4 matrixObj(
        matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], matPose.m[3][0],
        matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], matPose.m[3][1],
        matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], matPose.m[3][2],
        matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], matPose.m[3][3]);
    return matrixObj;
}

glm::mat4 VRSystem::convertSteamVRMatrixToGLM(const vr::HmdMatrix34_t &matPose) {
    glm::mat4 matrixObj(
        matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], 0.0,
        matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], 0.0,
        matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], 0.0,
        matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], 1.0f);
    return matrixObj;
}

void VRSystem::createVrEntities(Scene* activeScene, entt::entity par) {
    Entity HMD = activeScene->CreateEntity("HMD");
    auto &transform = HMD.GetComponent<TransformComponent>();
    transform.setParent(Entity(par, activeScene));
    transform.setRotation(glm::vec3(glm::radians(90.0f), 0.0f, glm::radians(-90.0f)));
    transform.setTranslation(glm::vec3(0.0f, 0.0f, 0.0f));

    createVrEntity(activeScene, HMD, vr::Eye_Left);
    createVrEntity(activeScene, HMD, vr::Eye_Right);
}

void VRSystem::createVrEntity(Scene* activeScene, Entity parent, vr::EVREye eye) {
    Entity vrCameraEntity = activeScene->CreateEntity("VRCamera" + eye);
    vrCameraEntity.AddComponent<CameraComponent>();
    vrCameraEntity.GetComponent<TransformComponent>().setParent(parent);

    Entity vrFramebufferEntity = activeScene->CreateEntity("VREntity" + eye);
    vrFramebufferEntity.AddComponent<VRComponent>(eye);
    auto& framebufferLeft = vrFramebufferEntity.AddComponent<FrameBufferComponent>();
    _pHMD->GetRecommendedRenderTargetSize(&framebufferLeft.RenderWidth, &framebufferLeft.RenderHeight);

    // Give framebuffer a camera entity to specify view to render
    framebufferLeft.CameraEntity = vrCameraEntity.GetHandle();
    // Add mesh to specify texture for Framebuffer to render to
    Mesh mesh = Mesh::nonEmptyMesh();
    mesh.Textures.push_back(Texture(framebufferLeft.RenderWidth, framebufferLeft.RenderHeight,
         "", GL_TEXTURE_2D, GL_RGBA));
    auto& renderable = vrFramebufferEntity.AddComponent<RenderableElementComponent>(0, mesh);
    renderable.StaticShow = false;
}
