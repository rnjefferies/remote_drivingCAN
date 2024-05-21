// Copyright 2021 Hoffmann
#include "Polygon.h"

namespace TodStandardEntities {

Entity Polygon::create(std::shared_ptr<Scene> scene, const std::string &name, const std::string& packagePath) {
    Entity entity = scene->CreateEntity(name);
    auto& transform = entity.GetComponent<TransformComponent>();
    unsigned int shaderProgram = ShaderSystem::createShaderProgram(
        (packagePath + "/resources/OpenGL/shaders/shader.vert").c_str(),
        (packagePath + "/resources/OpenGL/shaders/shaderTransparent.frag").c_str());
    Mesh mesh = Mesh::nonEmptyMesh();
    mesh.Indices = { 0, 1, 3, 1, 2, 3 };
    auto& renderable = entity.AddComponent<RenderableElementComponent>(
        shaderProgram, mesh, GL_TRIANGLES);
    entity.AddComponent<ExpirableComponent>(4000);
    transform.setTranslation(glm::vec3(0.0f, 0.0f, 0.025f)); // move up
    entity.AddComponent<DynamicDataComponent>();
    return entity;
}

void Polygon::onPolygonReceived(
    const geometry_msgs::PolygonStampedConstPtr& msg, Entity &polygonEntity,
    const std::map<std::string, Entity> &coordinateSystems) {
    tod_msgs::ColoredPolygonPtr coloredPolygonMsg = boost::make_shared<tod_msgs::ColoredPolygon>();
    coloredPolygonMsg->header = msg->header;
    for (const auto &point : msg->polygon.points) {
        auto &coloredPoint = coloredPolygonMsg->points.emplace_back();
        coloredPoint.point = point;
        coloredPoint.color.g = 1.0f;
    }
    onColoredPolygonReceived(coloredPolygonMsg, polygonEntity, coordinateSystems);
}

void Polygon::onColoredPolygonReceived(
    const tod_msgs::ColoredPolygonConstPtr& msg, Entity &polygonEntity,
    const std::map<std::string, Entity> &coordinateSystems) {
    auto &renderable = polygonEntity.GetComponent<RenderableElementComponent>();
    auto &expirable = polygonEntity.GetComponent<ExpirableComponent>();
    auto &dynamic = polygonEntity.GetComponent<DynamicDataComponent>();
    std::lock_guard<std::mutex> lock(*dynamic.Mutex);
    polygonEntity.GetComponent<TransformComponent>().setParent(coordinateSystems.at(msg->header.frame_id));
    expirable.restamp();
    auto &mesh = renderable.Meshes.front();
    mesh.Vertices.clear();
    mesh.Indices.clear();
    for (const auto &coloredPoint : msg->points) {
        const auto &pt = coloredPoint.point;
        const auto &cl = coloredPoint.color;
        mesh.Vertices.emplace_back(glm::vec3(pt.x, pt.y, pt.z), glm::vec2(), glm::vec3(cl.r, cl.g, cl.b));
    }
    triangulation(mesh.Vertices.size(), mesh.Indices);
    dynamic.HasNewData = true;
}

// Probably only works well for path-like Polygon
void Polygon::triangulation(int size, std::vector<unsigned int>& indices) {
    int i_end = (size/2) -1;
    for (int i = 0; i < i_end; i++) {
        indices.push_back(i);
        indices.push_back(i+1);
        indices.push_back(size-(i+1));
        indices.push_back(size-(i+1));
        indices.push_back(size-(i+2));
        indices.push_back(i+1);
    }
}

}; // namespace TodStandardEntities
