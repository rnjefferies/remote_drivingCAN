// Copyright 2020 Hoffmann
#pragma once
#include "Scene/Components.h"
#include "Scene/Entity.h"
#include "Scene/Scene.h"
#include "Systems/ShaderSystem.h"
#include "Core/RosInterface.h"
#include <string>
#include <memory>
#include <vector>

namespace TodStandardEntities {

class TopView {
public:
    static Entity create(std::shared_ptr<Scene> scene, const std::string &name, Entity parent);

private:
    TopView() = default;
};

}; // namespace TodStandardEntities
