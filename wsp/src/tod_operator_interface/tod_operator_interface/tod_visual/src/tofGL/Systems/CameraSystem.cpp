// Copyright 2021 Hoffmann
#include "Systems/CameraSystem.h"
#include "Systems/TransformSystem.h"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"

void CameraSystem::CalcViewMatrix(CameraComponent& camera, TransformComponent& transform) {
    glm::mat4 CameraTransform = TransformSystem::LocalToWorld(transform);
    glm::mat4 CameraRotation = TransformSystem::LocalToWorldRotation(transform);
    UpdatePositionFromLookAtAndRadius(camera);
    glm::vec3 viewPosition = glm::vec3(CameraTransform * glm::vec4(camera.Position, 1.0));
    glm::vec3 viewlookAt = glm::vec3(CameraTransform * glm::vec4(camera.LookAt, 1.0));
    glm::vec3 viewUp =  glm::vec3(CameraRotation * glm::vec4(camera.Up, 1.0)); // only rotation required
    camera.View = glm::lookAt(viewPosition, viewlookAt, viewUp);
}

void CameraSystem::OnWindowSizeChanged(CameraComponent& camera, int width, int height) {
    camera.Projection = glm::perspective(glm::radians(camera.FieldOfView),
                                         (float)width / (float) height, camera.NearPlane, camera.FarPlane);
}

void CameraSystem::UpdatePositionFromLookAtAndRadius(CameraComponent& camera) {
    camera.Position.x = camera.LookAt.x - (camera.Radius + camera.LookAt.x) * std::cos(glm::radians(camera.Yaw));
    camera.Position.y = (camera.Radius + camera.LookAt.x) * std::sin(glm::radians(camera.Yaw));
    glm::vec3 cameraLookDirection = glm::normalize(camera.Position-camera.LookAt);
    glm::vec3 worldUp{0.0f, 0.0f, 1.0f};
    glm::vec3 cameraRight = glm::normalize(glm::cross(worldUp, cameraLookDirection));
    camera.Up = glm::normalize(glm::cross(cameraLookDirection, cameraRight));
}
