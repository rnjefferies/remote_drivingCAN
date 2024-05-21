// Copyright 2021 Schimpe
#include "Path.h"

namespace TodStandardEntities {

Entity Path::create(std::shared_ptr<Scene> scene, const std::string &name,
                    const std::string& packagePath) {
    Entity laneEntity = scene->CreateEntity(name);
    unsigned int shaderProgram = ShaderSystem::createShaderProgram(
        (packagePath + "/resources/OpenGL/shaders/shader.vert").c_str(),
        (packagePath + "/resources/OpenGL/shaders/shader.frag").c_str());
    Mesh mesh = Mesh::nonEmptyMesh();
    auto& renderable = laneEntity.AddComponent<RenderableElementComponent>(shaderProgram, mesh, GL_LINE_STRIP);
    renderable.LineWidth = 5.0f;
    renderable.PointSize = 7.0f;
    laneEntity.AddComponent<ExpirableComponent>(1000);
    laneEntity.AddComponent<DynamicDataComponent>();
    auto& transform = laneEntity.GetComponent<TransformComponent>();
    transform.setTranslation(glm::vec3(0.0f, 0.0f, 0.05f)); // move up to put on top of videos
    return laneEntity;
}

void Path::onPathReceived(const nav_msgs::PathConstPtr& msg,
                          Entity &pathEntity, const std::map<std::string, Entity> &coordinateSystems) {
    auto &dynamic = pathEntity.GetComponent<DynamicDataComponent>();
    std::lock_guard<std::mutex> lock(*dynamic.Mutex);
    pathEntity.GetComponent<TransformComponent>().setParent(coordinateSystems.at(msg->header.frame_id));
    auto &renderable = pathEntity.GetComponent<RenderableElementComponent>();
    auto &expirable = pathEntity.GetComponent<ExpirableComponent>();
    expirable.restamp();
    auto &mesh = renderable.Meshes.front();
    mesh.Vertices.clear();
    //    for (int i = 0; i != std::min(15, (int) msg->poses.size()); ++i) {
    for (int i = 0; i != msg->poses.size(); ++i) {
        const auto& pos = msg->poses.at(i).pose.position;
        mesh.Vertices.emplace_back(glm::vec3((float) pos.x, (float) pos.y, (float) pos.z),
                                   glm::vec2(0.0f, 0.0f), glm::vec3(1.0f, 1.0f, 1.0f));
    }
    dynamic.HasNewData = true;
}

}; // namespace TodStandardEntities
