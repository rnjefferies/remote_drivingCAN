// Copyright 2021 Schimpe
#pragma once
#include "Scene/Components.h"
#include "Scene/Entity.h"
#include "Scene/Scene.h"
#include "Systems/ShaderSystem.h"
#include "Core/RosInterface.h"
#include <future>
#include <memory>
#include <string>
#include <algorithm>
#include <vector>
#include <tod_helper/camera_models/PinholeModel.h>
#include <tod_helper/camera_models/OcamModel.h>

namespace TodStandardEntities {

class Video {
private:
    Video() = default;
    static constexpr float SPHERE_MESH_INCREMENT{0.02f}; // used for angle and radius steps

public:
    static Entity create(std::shared_ptr<Scene> scene, const std::string &camName, const bool isFisheye, Entity parent);
    static void onImageReceived(const sensor_msgs::ImageConstPtr& msg, Entity &videoEntity);
    static void onProjectionReceived(const sensor_msgs::ImageConstPtr& msg, Entity &videoEntity);

    template<typename T>
    static void init_mesh(Entity &ent, const T &camMdl, const geometry_msgs::TransformStamped tf2cam);
    static void init_mesh_from_pinhole_model(Entity &ent, const PinholeModel &camMdl,
                                             const geometry_msgs::TransformStamped tf2cam);
    static void init_mesh_from_ocam_model(Entity &ent, const OcamModel &camMdl,
                                          const geometry_msgs::TransformStamped tf2cam);

    template <typename T>
    static void get_points_on_ground_plane
        (std::vector<std::vector<Vertex>> &rowsOnSphere, size_t &maxRowLength,
         const VideoComponent &video, const T& vidParams, const geometry_msgs::TransformStamped &tf);

    template <typename T>
    static void get_points_on_sphere
        (std::vector<std::vector<Vertex>> &rowsOnSphere, size_t &maxRowLength,
         const VideoComponent &video, const T& vidParams, const geometry_msgs::TransformStamped &tf);

    static void push_triangles_to_renderable(std::vector<std::vector<Vertex> > &rowsOnSphere,
                                             const size_t maxRowLength, RenderableElementComponent &renderable);
};

}; // namespace TodStandardEntities
