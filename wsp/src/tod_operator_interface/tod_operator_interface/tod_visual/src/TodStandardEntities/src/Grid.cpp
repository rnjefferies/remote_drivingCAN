// Copyright 2020 TUMFTM
#include "Grid.h"

namespace TodStandardEntities {

Entity Grid::create(std::shared_ptr<Scene> scene, const std::string &name, const std::string &packagePath) {
    Entity grid = scene->CreateEntity(name);
    unsigned int shader = ShaderSystem::createShaderProgram(
        (packagePath + "/resources/OpenGL/shaders/grid.vert").c_str(),
        (packagePath + "/resources/OpenGL/shaders/grid.frag").c_str());
    Mesh mesh = init_grid_mesh(1, 1000);
    grid.AddComponent<RenderableElementComponent>(shader, mesh, GL_LINES);
    return grid;
}

Mesh Grid::init_grid_mesh(const float gridSpacing, const int gridSize) {
    std::vector<Vertex> vertices;
    float xOffset = gridSize/2;
    float zPosition { -0.01f };
    for (int i = 0; i < gridSize; i++) {
        // Y Line
        vertices.emplace_back(glm::vec3(gridSpacing*(i - xOffset), gridSpacing*(gridSize / 2), zPosition),
                              glm::vec2(), glm::vec3(1.0f, 1.0f, 1.0f));
        vertices.emplace_back(glm::vec3(gridSpacing*(i - xOffset), gridSpacing*(-gridSize / 2), zPosition),
                              glm::vec2(), glm::vec3(1.0f, 1.0f, 1.0f));
        // X Line
        vertices.emplace_back(glm::vec3(gridSpacing*(-xOffset), gridSpacing*(i - gridSize / 2), zPosition),
                              glm::vec2(), glm::vec3(1.0f, 1.0f, 1.0f));
        vertices.emplace_back(glm::vec3(gridSpacing*(gridSize - xOffset), gridSpacing*(i - gridSize / 2), zPosition),
                              glm::vec2(), glm::vec3(1.0f, 1.0f, 1.0f));
    }
    return Mesh(vertices);
}

}; // namespace TodStandardEntities
