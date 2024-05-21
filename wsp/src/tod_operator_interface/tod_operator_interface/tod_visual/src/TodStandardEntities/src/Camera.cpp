// Copyright 2020 TUMFTM
#include "Camera.h"

namespace TodStandardEntities {

Entity Camera::create(std::shared_ptr<Scene> scene, std::string name, Entity parent) {
    Entity camera = scene->CreateEntity(name);
    camera.AddComponent<CameraComponent>(glm::perspective(glm::radians(45.0f),
                                                          (float)1280/(float)720, 0.1f, 50.0f), true);
    camera.GetComponent<TransformComponent>().setParent(parent);
    return camera;
}

void Camera::onGearUpdate(const tod_msgs::VehicleDataConstPtr& msg, Entity &entity) {
    static eGearPosition previousGear{eGearPosition::GEARPOSITION_PARK};
    if (msg->gearPosition == eGearPosition::GEARPOSITION_REVERSE &&
        previousGear != eGearPosition::GEARPOSITION_REVERSE) {
            entity.GetComponent<CameraComponent>().Yaw = 180.0f;
            entity.GetComponent<CameraComponent>().Radius = 0.0f;
    }
    if (msg->gearPosition != eGearPosition::GEARPOSITION_REVERSE &&
            previousGear == eGearPosition::GEARPOSITION_REVERSE) {
        entity.GetComponent<CameraComponent>().Yaw = 0.0f;
        entity.GetComponent<CameraComponent>().Radius = 1.5f;
    }
    previousGear = static_cast<eGearPosition>(msg->gearPosition);
}

}; // namespace TodStandardEntities
