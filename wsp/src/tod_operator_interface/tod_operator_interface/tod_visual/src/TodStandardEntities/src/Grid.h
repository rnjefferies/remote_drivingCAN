// Copyright 2020 TUMFTM
#pragma once
#include "Scene/Components.h"
#include "Scene/Entity.h"
#include "Scene/Scene.h"
#include "Systems/ShaderSystem.h"
#include <memory>
#include <string>
#include <vector>

namespace TodStandardEntities {

class Grid {
public:
    static Entity create(std::shared_ptr<Scene> scene, const std::string &name,
                         const std::string &packagePath);

private:
    Grid() = default;
    static Mesh init_grid_mesh(const float gridSpacing, const int gridSize);
};

}; // namespace TodStandardEntities
