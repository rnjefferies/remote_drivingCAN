// Copyright 2020 TUMFTM
#pragma once
#include "Scene/Components.h"
#include "Scene/Entity.h"
#include "Scene/Scene.h"
#include <string>
#include <memory>
#include "tod_msgs/VehicleData.h"
#include "tod_msgs/VehicleEnums.h"

namespace TodStandardEntities {

class Camera {
private:
    Camera() = default;

public:
    static Entity create(std::shared_ptr<Scene> scene, std::string name, Entity parent);
    static void onGearUpdate(const tod_msgs::VehicleDataConstPtr& msg, Entity &entity);
};

}; // namespace TodStandardEntities
