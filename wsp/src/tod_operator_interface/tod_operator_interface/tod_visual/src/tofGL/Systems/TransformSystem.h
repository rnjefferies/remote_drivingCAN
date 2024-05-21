// Copyright 2021 Hoffmann
#pragma once
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"
#include "Scene/Components.h"

class TransformSystem {
public:
    ~TransformSystem();
    static glm::mat4 LocalToWorld(TransformComponent& transform);
    static glm::mat4 LocalToWorldRotation(TransformComponent &transform);

private:
    TransformSystem();
    static glm::mat4 LocalToParent(TransformComponent& transform);
    static glm::mat4 LocalToParentRotation(TransformComponent &transform);
};
