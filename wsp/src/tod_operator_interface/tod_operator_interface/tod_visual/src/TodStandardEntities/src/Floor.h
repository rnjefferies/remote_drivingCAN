// Copyright 2020 Feiler
#pragma once
#include "Scene/Components.h"
#include "Scene/Entity.h"
#include "Scene/Scene.h"
#include "Systems/ShaderSystem.h"
#include <memory>
#include <string>
#include <vector>

namespace TodStandardEntities {

class Floor {
public:
    static Entity create(std::shared_ptr<Scene> scene, const std::string &name,
                         const std::string &packagePath, const Entity &parent);

private:
    Floor() = default;
};

}; // namespace TodStandardEntities
