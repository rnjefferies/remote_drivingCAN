// Copyright 2021 Hoffmann
#include "Systems/TransformSystem.h"

glm::mat4 TransformSystem::LocalToWorld(TransformComponent &transform) {
    if (transform.ParentEntity.GetHandle() == entt::null) {
        return LocalToParent(transform);
    } else {
        auto parentTransform = transform.ParentEntity.GetComponent<TransformComponent>();
        parentTransform.Scale = glm::vec3{1.0f, 1.0f, 1.0f};
        return LocalToWorld(parentTransform) * LocalToParent(transform);
    }
}

glm::mat4 TransformSystem::LocalToParent(TransformComponent& transform) {
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), transform.Rotation.z, { 0, 0, 1 })
                         * glm::rotate(glm::mat4(1.0f), transform.Rotation.y, { 0, 1, 0 })
                         * glm::rotate(glm::mat4(1.0f), transform.Rotation.x, { 1, 0, 0 });

    return glm::translate(glm::mat4(1.0f), transform.Translation)
           * rotation
           * glm::scale(glm::mat4(1.0f), transform.Scale);
}

glm::mat4 TransformSystem::LocalToWorldRotation(TransformComponent &transform) {
    if (transform.ParentEntity.GetHandle() == entt::null) {
        return LocalToParentRotation(transform);
    } else {
        return LocalToWorldRotation(transform.ParentEntity.GetComponent<TransformComponent>())
               * LocalToParentRotation(transform);
    }
}

glm::mat4 TransformSystem::LocalToParentRotation(TransformComponent &transform) {
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), transform.Rotation.x, { 1, 0, 0 })
                         * glm::rotate(glm::mat4(1.0f), transform.Rotation.y, { 0, 1, 0 })
                         * glm::rotate(glm::mat4(1.0f), transform.Rotation.z, { 0, 0, 1 });

    return rotation;
}
