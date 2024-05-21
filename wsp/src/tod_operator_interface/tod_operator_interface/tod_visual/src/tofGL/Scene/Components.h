// Copyright 2021 TUMFTM
#pragma once
#include <sensor_msgs/Image.h>
#include <openvr/openvr.h>
#include <map>
#include <mutex>
#include <memory>
#include <vector>
#include <string>
#include "Entity.h"
#include <Core/DataContainer.h>

struct TagComponent {
    std::string Tag{""};

    TagComponent() = default;
    TagComponent(const TagComponent&) = default;
    explicit TagComponent(const std::string &tag) : Tag(tag) {}
};

struct TransformComponent {
    glm::vec3 Translation{0.0f, 0.0f, 0.0f};
    glm::vec3 Rotation{0.0f, 0.0f, 0.0f};
    glm::vec3 Scale{1.0f, 1.0f, 1.0f};
    Entity ParentEntity;

    TransformComponent() = default;
    TransformComponent(const TransformComponent&) = default;

    void setRotation(const glm::vec3 &rotation) { Rotation = rotation; }
    void setTranslation(const glm::vec3 &translation) { Translation = translation; }
    void setScale(const glm::vec3 &scale) { Scale = scale; }
    void setParent(const Entity &entity) { ParentEntity = entity; }
};

struct RenderableElementComponent {
    unsigned int ShaderProgram;
    std::vector<Mesh> Meshes;
    GLenum RenderMode;
    float LineWidth{1.0f}, PointSize{1.0f};
    bool StaticShow{true}, DynamicShow{true};

    RenderableElementComponent(const unsigned int shaderProgram, const std::vector<Mesh> &meshes,
                               const GLenum renderMode = GL_TRIANGLES)
        : ShaderProgram(shaderProgram), Meshes(meshes), RenderMode(renderMode) {
        for (Mesh &mesh : Meshes) {
            if (mesh.Vertices.empty()) {
                ROS_WARN("%s: RenderableElementComponent(MeshVector) - empty vertices - filling up",
                         ros::this_node::getName().c_str());
                mesh.Vertices = Mesh::VectorOfThreeZeroVertices();
            }
        }
    }

    RenderableElementComponent(const unsigned int shaderProgram, const Mesh &mesh,
                               GLenum renderMode = GL_TRIANGLES)
        : ShaderProgram(shaderProgram), RenderMode(renderMode) {
        Meshes.emplace_back(mesh);
        if (mesh.Vertices.empty()) {
            ROS_WARN("%s: RenderableElementComponent(Mesh) - empty vertices  - filling up"
                     " - filling up with 3x (0,0,0)", ros::this_node::getName().c_str());
            Meshes.back().Vertices = Mesh::VectorOfThreeZeroVertices();
        }
    }
};

struct DynamicDataComponent {
    bool HasNewData{false};
    std::shared_ptr<std::mutex> Mutex{std::make_shared<std::mutex>()};

    DynamicDataComponent() = default;
    DynamicDataComponent(const DynamicDataComponent&) = default;
};

struct ExpirableComponent {
    uint64_t StampMs{0};
    uint64_t TimeToExpireMs{0};

    ExpirableComponent() = default;
    ExpirableComponent(const ExpirableComponent&) = default;
    explicit ExpirableComponent(const uint64_t &myTimeToExpireMs) : TimeToExpireMs(myTimeToExpireMs) {}

    uint64_t now() { return std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1000000; }
    bool expired() { return (now() >= (StampMs + TimeToExpireMs)); }
    void restamp() { StampMs = now(); }
};

struct VideoComponent {
    enum ProjectionModeType {
        RECTANGULAR = 0,
        SPHERE = 1,
        HALF_SPHERE_WITH_GROUND_PLANE = 2,
        GROUND_PLANE = 3,
        ROBINSON = 4
    };

    struct PixelBuffer {
        Buffer Buf;
        unsigned int Width, Height;
        std::string Name;
        PixelBuffer(const unsigned int width, const unsigned int height, const std::string &name)
            : Buf(GL_PIXEL_UNPACK_BUFFER, GL_DYNAMIC_DRAW), Width{width}, Height{height}, Name{name} {}
    };

    std::string CameraName{""};
    bool IsFisheye{false};

    std::vector<PixelBuffer> PixelBuffers;
    std::shared_ptr<std::mutex> Mutex{std::make_shared<std::mutex>()};
    sensor_msgs::ImageConstPtr ImageMsg{nullptr};
    sensor_msgs::ImageConstPtr ProjectionMsg{nullptr};

    ProjectionModeType ProjectionMode{ProjectionModeType::RECTANGULAR};
    float GroundPlaneRadiusMin{0.01f};
    float SphereRadius{15.0f}; // also used as GroundPlaneRadiusMax
    float SphereLongitudeMin{-3.1415f}, SphereLongitudeMax{+3.1415f};
    float SphereLatitudeMin{-1.5708f}, SphereLatitudeMax{+1.0472f};

    int WidthRaw{0}, HeightRaw{0};
    float ScalingX{1.0}, ScalingY{1.0};

    VideoComponent() = default;
    VideoComponent(const VideoComponent&) = default;
    VideoComponent(const std::string &cameraName, const bool isFisheye)
        : CameraName(cameraName), IsFisheye{isFisheye} {}
};

struct CameraComponent {
    glm::mat4 Projection;
    glm::mat4 View;
    float Yaw{0.0f};
    float Radius{1.5f};
    glm::vec3 LookAt{3.0f, 0.0f, 1.0f};
    glm::vec3 Position{0.0f, 0.0f, 2.0f};
    glm::vec3 Up{0.0f, 0.0f, 1.0f};
    bool Controllable{false};
    float NearPlane{0.1f};
    float FarPlane{100.0f};
    float FieldOfView{45.0f};

    CameraComponent() = default;
    CameraComponent(const CameraComponent&) = default;
    CameraComponent(const glm::mat4 &projection, bool controllable)
        : Projection(projection), Controllable(controllable) {}
};

struct VRComponent {
    vr::Hmd_Eye Eye;

    VRComponent() = default;
    VRComponent(const VRComponent&) = default;
    explicit VRComponent(const vr::Hmd_Eye &eye) : Eye(eye) {}
};

struct FrameBufferComponent {
    GLuint RenderTextureId;
    unsigned int fbo{0}, rbo{0};

    entt::entity CameraEntity;
    int samples{0};

    bool IsDefaultFramebuffer{false};
    unsigned int RenderWidth{1280};
    unsigned int RenderHeight{1280};

    FrameBufferComponent() = default;
    FrameBufferComponent(const FrameBufferComponent&) = default;
    explicit FrameBufferComponent(bool isDefaultFramebuffer) : IsDefaultFramebuffer(isDefaultFramebuffer) {}
};

struct CharacterMapComponent {
    std::map<char, Character> Characters;
    float PixelPerMeterRatio{ 1000.0f };

    CharacterMapComponent() = default;
    CharacterMapComponent(const CharacterMapComponent&) = default;
};
