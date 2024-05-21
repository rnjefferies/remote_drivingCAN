// Copyright 2021 Schimpe
#pragma once
#include "Scene/Components.h"
#include "Scene/Entity.h"
#include "Scene/Scene.h"
#include "Systems/ShaderSystem.h"
#include <nav_msgs/Path.h>
#include <string>
#include <map>
#include <memory>
#include <algorithm>

namespace TodStandardEntities {

class Path {
public:
    static Entity create(std::shared_ptr<Scene> scene, const std::string &name,
                         const std::string& packagePath);
    static void onPathReceived(const nav_msgs::PathConstPtr& msg, Entity &pathEntity,
                               const std::map<std::string, Entity> &coordinateSystems);

private:
    Path() = default;
};

}; // namespace TodStandardEntities
