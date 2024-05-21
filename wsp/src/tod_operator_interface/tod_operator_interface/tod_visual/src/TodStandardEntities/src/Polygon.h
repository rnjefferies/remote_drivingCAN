// Copyright 2021 Hoffmann
#pragma once
#include "Scene/Components.h"
#include "Scene/Entity.h"
#include "Scene/Scene.h"
#include "Systems/ShaderSystem.h"
#include "geometry_msgs/PolygonStamped.h"
#include "tod_msgs/ColoredPolygon.h"
#include <string>
#include <map>
#include <memory>
#include <vector>

namespace TodStandardEntities {

class Polygon {
private:
    Polygon() = default;

public:
    static Entity create(std::shared_ptr<Scene> scene, const std::string &name,
                         const std::string& packagePath);
    static void onPolygonReceived(const geometry_msgs::PolygonStampedConstPtr& msg, Entity &pathEntity,
                                  const std::map<std::string, Entity> &coordinateSystems);
    static void onColoredPolygonReceived(const tod_msgs::ColoredPolygonConstPtr& msg, Entity &polygonEntity,
                                         const std::map<std::string, Entity> &coordinateSystems);
    static void triangulation(int size, std::vector<unsigned int>& indices);
};

}; // namespace TodStandardEntities
