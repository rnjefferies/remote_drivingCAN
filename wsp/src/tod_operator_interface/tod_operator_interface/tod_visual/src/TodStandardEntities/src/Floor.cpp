// Copyright 2020 Feiler
#include "Floor.h"

namespace TodStandardEntities {

Entity Floor::create(std::shared_ptr<Scene> scene, const std::string &name,
                     const std::string &packagePath, const Entity &parent) {
    Entity floorFor3DMouseClick = scene->CreateEntity(name);
    unsigned int grid_shader = ShaderSystem::createShaderProgram(
        (packagePath + "/resources/OpenGL/shaders/shader.vert").c_str(),
        (packagePath + "/resources/OpenGL/shaders/shader.frag").c_str());
    std::vector<Vertex> vertices;
    float zPosition { -0.02f };
    vertices.emplace_back(glm::vec3(100.0f, -100.0f, zPosition));
    vertices.emplace_back(glm::vec3(-100.0f, 100.0f, zPosition));
    vertices.emplace_back(glm::vec3(100.0f, 100.0f, zPosition));
    vertices.emplace_back(glm::vec3(100.0f, -100.0f, zPosition));
    vertices.emplace_back(glm::vec3(-100.0f, -100.0f, zPosition));
    vertices.emplace_back(glm::vec3(-100.0f, 100.0f, zPosition));
    Mesh mesh(vertices);
    floorFor3DMouseClick.AddComponent<RenderableElementComponent>(grid_shader, mesh);
    floorFor3DMouseClick.GetComponent<TransformComponent>().setTranslation(glm::vec3(0.0f, 0.0f, -0.02f));
    floorFor3DMouseClick.GetComponent<TransformComponent>().setParent(parent);
    return floorFor3DMouseClick;
}

}; // namespace TodStandardEntities
