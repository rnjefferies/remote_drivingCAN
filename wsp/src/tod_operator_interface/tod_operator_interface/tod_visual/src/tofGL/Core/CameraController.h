// Copyright 2020 TUMFTM
#pragma once
#include "Scene/Components.h"
#include "Events/KeyEvent.h"
#include "Events/MouseCodes.h"
#include "Events/MouseEvent.h"
#include "Events/KeyCodes.h"
#include "Events/ApplicationEvent.h"
#include "Systems/CameraSystem.h"

enum CameraMovement {
    YAW_INCREASE,
    YAW_DECREASE,
    RADIUS_INCREASE,
    RADIUS_DECREASE,
    Z_POS_INCREASE,
    Z_POS_DECREASE,
    Z_LOOKAT_INCREASE,
    Z_LOOKAT_DECREASE
};

class CameraController {
    public:
        CameraController() = default;
        ~CameraController() = default;
        void onEvent(Event& e, CameraComponent& camera, float delta_time);

    private:
        void moveCamera(float deltaTime, CameraMovement direction, CameraComponent& camera);
};
