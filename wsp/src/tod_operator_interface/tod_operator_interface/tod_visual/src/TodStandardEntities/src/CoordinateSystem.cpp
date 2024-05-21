// Copyright 2020 TUMFTM
#include "CoordinateSystem.h"

namespace TodStandardEntities {

Entity CoordinateSystem::create(std::shared_ptr<Scene> scene, const std::string &name) {
    Entity coordinateSystem = scene->CreateEntity(name);
    unsigned int grid_shader = ShaderSystem::createShaderProgram(
        (RosInterface::getPackagePath() + "/resources/OpenGL/shaders/shader.vert").c_str(),
        (RosInterface::getPackagePath() + "/resources/OpenGL/shaders/shader.frag").c_str());
    std::vector<Vertex> vertices;
    vertices.emplace_back(glm::vec3(0.00f, 0.0f, 0.0f), glm::vec2(), glm::vec3(1.0f, 0.0f, 0.0f));
    vertices.emplace_back(glm::vec3(0.25f, 0.0f, 0.0f), glm::vec2(), glm::vec3(1.0f, 0.0f, 0.0f));
    vertices.emplace_back(glm::vec3(0.0f, 0.00f, 0.0f), glm::vec2(), glm::vec3(0.0f, 1.0f, 0.0f));
    vertices.emplace_back(glm::vec3(0.0f, 0.25f, 0.0f), glm::vec2(), glm::vec3(0.0f, 1.0f, 0.0f));
    vertices.emplace_back(glm::vec3(0.0f, 0.0f, 0.00f), glm::vec2(), glm::vec3(0.0f, 0.0f, 1.0f));
    vertices.emplace_back(glm::vec3(0.0f, 0.0f, 0.25f), glm::vec2(), glm::vec3(0.0f, 0.0f, 1.0f));
    coordinateSystem.AddComponent<RenderableElementComponent>(grid_shader, Mesh(vertices), GL_LINES);
    coordinateSystem.GetComponent<RenderableElementComponent>().LineWidth = 5.0f;
    return coordinateSystem;
}

} // namespace TodStandardEntities
