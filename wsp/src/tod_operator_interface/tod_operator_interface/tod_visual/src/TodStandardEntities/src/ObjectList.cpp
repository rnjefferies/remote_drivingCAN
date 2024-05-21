// Copyright 2021
#include "ObjectList.h"

namespace TodStandardEntities {

Entity ObjectList::create(std::shared_ptr<Scene> scene,
            std::string name, const std::string& packagePath) {
    Entity objectList = scene->CreateEntity(name);
    unsigned int shaderProgram = ShaderSystem::createShaderProgram(
                (packagePath + "/resources/OpenGL/shaders/shader.vert").c_str(),
                // (packagePath + "/resources/OpenGL/shaders/shader.frag").c_str());
                (packagePath + "/resources/OpenGL/shaders/shaderTransparent.frag").c_str());
    int expectedNumberOfObjects { 50 };
    std::vector<unsigned int> indices {
        0, 1, 2, 0, 2, 3,
        0, 3, 4, 3, 4, 7,
        4, 5, 6, 4, 6, 7,
        1, 2, 5, 2, 5, 6,
        0, 1, 5, 0, 5, 4,
        3, 2, 6, 3, 6, 7
        };
    std::vector<unsigned int> initialIndexVec;
    int numberOfVerticesPerObject{ 8 };
    std::vector<Vertex> vertices;
    for ( int expectedObjectsIterator = 0;
            expectedObjectsIterator != expectedNumberOfObjects;
            ++expectedObjectsIterator) {
        for ( int iterator = 0; iterator != numberOfVerticesPerObject; ++iterator ) {
            vertices.push_back(Vertex());
        }
        initialIndexVec.insert(std::end(initialIndexVec), std::begin(indices),
                std::end(indices));
    }
    Mesh mesh(vertices, indices);
    objectList.AddComponent<RenderableElementComponent>(shaderProgram, mesh);
    objectList.AddComponent<DynamicDataComponent>();
    objectList.AddComponent<ExpirableComponent>(1000);
    return objectList;
}

void ObjectList::processAndStoreMsgIntoObjectListEntity(
    const tod_msgs::ObjectListConstPtr& msg, Entity& objectListEntity,
    const std::map<std::string, Entity> &coordinateSystems) {

    auto& renderable = objectListEntity.GetComponent<RenderableElementComponent>();
    auto& expirable = objectListEntity.GetComponent<ExpirableComponent>();
    auto& dynamic = objectListEntity.GetComponent<DynamicDataComponent>();
    std::lock_guard<std::mutex> lock(*dynamic.Mutex);
    objectListEntity.GetComponent<TransformComponent>().setParent(coordinateSystems.at(msg->header.frame_id));
    dynamic.HasNewData = true;
    expirable.restamp();

    // reset vertex data;
    std::for_each(renderable.Meshes.begin(), renderable.Meshes.end(),
                  [](Mesh& mesh) {
                      mesh.Vertices.clear();
                      mesh.Indices.clear();
                  });

    for (unsigned int index = 0; index != msg->objectList.size(); ++index) {
        const auto& currentObject = msg->objectList.at(index);

        int numberOfVerticesPerObject{ 8 };
        for ( int iterator = 0; iterator != numberOfVerticesPerObject; ++iterator ) {
            renderable.Meshes.front().Vertices.push_back(Vertex());
        }

        glm::mat4 translation = glm::translate(glm::mat4(1.0f),
                                               glm::vec3(currentObject.distCenterX, currentObject.distCenterY, 0.0f));
        glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), currentObject.yawAngle,
                                         glm::vec3(0.0f, 0.0f, 1.0f));

        fillVerticesFromVectorsToTheObjectEdges(
            renderable.Meshes.front().Vertices,
            index, numberOfVerticesPerObject,
            glm::vec3(-currentObject.dimX/2, -currentObject.dimY/2, 0.0f),
            glm::vec3(-currentObject.dimX/2, currentObject.dimY/2, 0.0f),
            glm::vec3(currentObject.dimX/2, -currentObject.dimY/2, 0.0f),
            currentObject.dimZ);
        transformVertices(renderable.Meshes.front().Vertices,
                          index, numberOfVerticesPerObject,
                          translation, rotation);
        addIndices(renderable.Meshes.front().Indices, index, numberOfVerticesPerObject);

        // bounding box color
        glm::vec3 color;
        if ( currentObject.classification == tod_msgs::ObjectData::CLASSIFICATION_UNKNOWN ) {
            color = glm::vec3{1.0f, 1.0f, 0.0f}; // yellow
        } else if ( currentObject.classification == tod_msgs::ObjectData::CLASSIFICATION_VEHICLE ) {
            color = glm::vec3{0.0f, 0.0f, 1.0f}; // blue
        } else if ( currentObject.classification == tod_msgs::ObjectData::CLASSIFICATION_ANIMAL ) {
            color = glm::vec3{0.0f, 1.0f, 0.0f}; // green
        } else if ( currentObject.classification == tod_msgs::ObjectData::CLASSIFICATION_VRU ) {
            color = glm::vec3{1.0f, 0.0f, 0.0f}; // red
        } else if ( currentObject.classification == tod_msgs::ObjectData::CLASSIFICATION_OTHER ) {
            color = glm::vec3{0.0f, 1.0f, 1.0f}; // cyan
        }
        for (int iterator = index*numberOfVerticesPerObject;
             iterator != renderable.Meshes.front().Vertices.size(); ++iterator) {
            renderable.Meshes.front().Vertices.at(iterator).TexColor = color;
        }
    }
}

//        6  ---  7
//      /       /
//    2  ---  3
//
//        5  ---  4
//     /       /
//    1  ---  0
void ObjectList::fillVerticesFromVectorsToTheObjectEdges(std::vector<Vertex>& vertices,
                                                               const int index, const int numberOfVerticesPerObject,
                                                               const glm::vec3& rearRight,
                                                               const glm::vec3& rearLeft,
                                                               const glm::vec3& frontRight,
                                                               const float height) {
    glm::vec3 frontLeft{ rearLeft + frontRight - rearRight };
    glm::vec3 heightVector = glm::vec3(0.0f, 0.0f, height);
    glm::vec3 rearRightHeight(rearRight + heightVector);
    glm::vec3 rearLeftHeight(rearLeft + heightVector);
    glm::vec3 frontLeftHeight(frontLeft + heightVector);
    glm::vec3 frontRightHeight(frontRight + heightVector);
    vertices.at(index*numberOfVerticesPerObject + 0) = Vertex(rearRight);
    vertices.at(index*numberOfVerticesPerObject + 1) = Vertex(rearLeft);
    vertices.at(index*numberOfVerticesPerObject + 2) = Vertex(rearLeftHeight);
    vertices.at(index*numberOfVerticesPerObject + 3) = Vertex(rearRightHeight);
    vertices.at(index*numberOfVerticesPerObject + 4) = Vertex(frontRight);
    vertices.at(index*numberOfVerticesPerObject + 5) = Vertex(frontLeft);
    vertices.at(index*numberOfVerticesPerObject + 6) = Vertex(frontLeftHeight);
    vertices.at(index*numberOfVerticesPerObject + 7) = Vertex(frontRightHeight);
}

void ObjectList::transformVertices(std::vector<Vertex>& vertices,
                                         const int index, const int numberOfVerticesPerObject,
                                         const glm::mat4& translation, const glm::mat4& rotation) {
    std::for_each(vertices.begin() + index*numberOfVerticesPerObject, vertices.end(),
                  [&rotation, &translation](Vertex& vertex){
                      glm::vec4 tmpVector =
                          translation * rotation * glm::vec4(vertex.Position, 1.0f);
                      vertex.Position = glm::vec3(tmpVector.x, tmpVector.y, tmpVector.z);
                  });
}

void ObjectList::addIndices(std::vector<unsigned int> &indices,
                                  const unsigned int index, const int numberOfVerticesPerObject) {
    unsigned int incre{ index*numberOfVerticesPerObject };
    std::vector<unsigned int> indicesOfAppendedObject {
        incre+0, incre+1, incre+2, incre+0, incre+2, incre+3,
        incre+0, incre+3, incre+4, incre+3, incre+4, incre+7,
        incre+4, incre+5, incre+6, incre+4, incre+6, incre+7,
        incre+1, incre+2, incre+5, incre+2, incre+5, incre+6,
        incre+0, incre+1, incre+5, incre+0, incre+5, incre+4,
        incre+3, incre+2, incre+6, incre+3, incre+6, incre+7
    };
    indices.insert(std::end(indices), std::begin(indicesOfAppendedObject),
                   std::end(indicesOfAppendedObject));
}

}   // namespace TodStandardEntities
