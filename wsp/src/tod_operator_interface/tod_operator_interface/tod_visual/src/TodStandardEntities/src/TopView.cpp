// Copyright 2020 Hoffmann
#include "TopView.h"

namespace TodStandardEntities {

Entity TopView::create(std::shared_ptr<Scene> scene, const std::string &name, Entity parent) {
    int maxResolution = 1280;
    float width = 2.0f;
    float height = 2.0f;

    Entity topViewEntity = scene->CreateEntity(name);
    unsigned int shader = ShaderSystem::createShaderProgram(
        (RosInterface::getPackagePath() + "/resources/OpenGL/shaders/topview.vert").c_str(),
        (RosInterface::getPackagePath() + "/resources/OpenGL/shaders/topview.frag").c_str());
    topViewEntity.GetComponent<TransformComponent>().setParent(parent);

    auto& transform = topViewEntity.GetComponent<TransformComponent>();
    transform.setScale(glm::vec3(1.0f, width, height));
    transform.setTranslation(glm::vec3(3.0f, 0.0f, 3.0f));

    auto& framebuffer = topViewEntity.AddComponent<FrameBufferComponent>();
    if (width > height) {
        framebuffer.RenderWidth = maxResolution;
        framebuffer.RenderHeight = (int)((float)maxResolution * height / width);
    } else {
        framebuffer.RenderHeight = maxResolution;
        framebuffer.RenderWidth = (int)((float)maxResolution * width / height);
    }

    std::vector<Vertex> vertices;
    vertices.emplace_back(
        glm::vec3(0.0f, -0.5f, 1.0f), glm::vec2(1.0f, 1.0f), glm::vec3(1.0f, 0.0f, 0.0f)); // topleft
    vertices.emplace_back(
        glm::vec3(0.0f, -0.5f, 0.0f), glm::vec2(1.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)); // bottomleft
    vertices.emplace_back(
        glm::vec3(0.0f, 0.5f, 0.0f), glm::vec2(0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)); // bottomright
    vertices.emplace_back(
        glm::vec3(0.0f, 0.5f, 1.0f), glm::vec2(0.0f, 1.0f), glm::vec3(1.0f, 0.0f, 0.0f)); // topright
    std::vector<unsigned int> indices{ 0, 1, 3, 1, 2, 3 };
    Mesh mesh(vertices, indices);
    mesh.Textures.emplace_back(framebuffer.RenderWidth, framebuffer.RenderHeight,
                               "texture_diffuse", GL_TEXTURE_2D, GL_RGB);

    auto& renderable = topViewEntity.AddComponent<RenderableElementComponent>(shader, mesh, GL_TRIANGLES);
    renderable.LineWidth = 5.0f;

    Entity TopViewCamera = scene->CreateEntity(name + "Camera");
    TopViewCamera.GetComponent<TransformComponent>().setParent(parent);
    auto& camera = TopViewCamera.AddComponent<CameraComponent>();
    camera.LookAt = glm::vec3(0.0f, 0.0f, 0.0f);
    camera.Position = glm::vec3(0.0f, 0.0f, 15.0f);
    camera.Radius = 0.00001f;

    framebuffer.CameraEntity = TopViewCamera.GetHandle();

    return topViewEntity;
}

}; // namespace TodStandardEntities
