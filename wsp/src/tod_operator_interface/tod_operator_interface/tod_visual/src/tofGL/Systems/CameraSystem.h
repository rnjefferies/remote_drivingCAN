// Copyright 2021 Hoffmann
#pragma once
#include "Scene/Components.h"

class CameraSystem {
public:
    ~CameraSystem();
    static void CalcViewMatrix(CameraComponent& camera, TransformComponent& transform);
    static void OnWindowSizeChanged(CameraComponent& camera, int width, int height);

private:
    CameraSystem();
    static void UpdatePositionFromLookAtAndRadius(CameraComponent& camera);
};
