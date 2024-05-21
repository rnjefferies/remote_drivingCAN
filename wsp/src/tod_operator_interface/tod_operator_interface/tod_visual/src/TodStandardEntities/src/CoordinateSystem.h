// Copyright 2020 TUMFTM
#pragma once
#include "Scene/Components.h"
#include "Scene/Entity.h"
#include "Scene/Scene.h"
#include "Systems/ShaderSystem.h"
#include "Core/RosInterface.h"
#include <string>
#include <vector>
#include <memory>

namespace TodStandardEntities {

class CoordinateSystem {
public:
    static Entity create(std::shared_ptr<Scene> scene, const std::string &name);

private:
    CoordinateSystem() = default;
};

} // namespace TodStandardEntities
