// Copyright 2021
#pragma once
#include "Scene/Entity.h"
#include "Scene/Components.h"
#include "Systems/ShaderSystem.h"
#include "Systems/Renderer.h"
#include <memory>
#include <string>
#include <map>
#include <vector>
#include "tod_msgs/ObjectList.h"

namespace TodStandardEntities {

class ObjectList {
    private:
        ObjectList() = default;

        static void fillVerticesFromVectorsToTheObjectEdges(
            std::vector<Vertex>& vertices,
            const int index, const int numberOfVerticesPerObject,
            const glm::vec3& rearRight,
            const glm::vec3& rearLeft,
            const glm::vec3& frontRight,
            const float height);
        static void transformVertices(
            std::vector<Vertex>& vertices,
            const int index, const int numberOfVerticesPerObject,
            const glm::mat4& translation, const glm::mat4& rotation);
        static void addIndices(std::vector<unsigned int> &indices,
                            const unsigned int index, const int numberOfVerticesPerObject);

    public:
        static Entity create(std::shared_ptr<Scene> scene,
            std::string name, const std::string& packagePath);
        static void processAndStoreMsgIntoObjectListEntity(
            const tod_msgs::ObjectListConstPtr& msg,
            Entity& objectListEntity, const std::map<std::string, Entity> &coordinateSystems);
};

}   // namespace TodStandardEntities
