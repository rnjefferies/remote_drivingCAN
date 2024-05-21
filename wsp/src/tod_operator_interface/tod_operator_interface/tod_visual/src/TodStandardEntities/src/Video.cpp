// Copyright 2021 Schimpe
#include "Video.h"

namespace TodStandardEntities {

Entity Video::create(std::shared_ptr<Scene> scene, const std::string &camName, const bool isFisheye,
                     Entity parent) {
    Entity videoEntity = scene->CreateEntity(camName);
    unsigned int videoShader = ShaderSystem::createShaderProgram(
        (RosInterface::getPackagePath() + "/resources/OpenGL/shaders/video.vert").c_str(),
        (RosInterface::getPackagePath() + "/resources/OpenGL/shaders/video.frag").c_str());
    auto &videoCmp = videoEntity.AddComponent<VideoComponent>(camName, isFisheye);
    // currently only i420 pixel format supported
    videoCmp.PixelBuffers.emplace_back(2048, 2048, "Ytex");
    videoCmp.PixelBuffers.emplace_back(2048, 2048 / 4, "Utex");
    videoCmp.PixelBuffers.emplace_back(2048, 2048 / 4, "Vtex");
    videoCmp.PixelBuffers.emplace_back(2048, 2048, "Ptex"); // mono8
    Mesh mesh = Mesh::nonEmptyMesh();
    for (auto& pxBuf : videoCmp.PixelBuffers)
        mesh.Textures.emplace_back(Texture(pxBuf.Width, pxBuf.Height, pxBuf.Name, GL_TEXTURE_RECTANGLE));
    videoEntity.AddComponent<RenderableElementComponent>(videoShader, mesh);
    videoEntity.AddComponent<ExpirableComponent>(1500);
    videoEntity.AddComponent<DynamicDataComponent>();
    videoEntity.GetComponent<TransformComponent>().setParent(parent);
    return videoEntity;
}

void Video::onImageReceived(const sensor_msgs::ImageConstPtr& msg, Entity &videoEntity) {
    auto &video = videoEntity.GetComponent<VideoComponent>();
    std::lock_guard<std::mutex> lock(*video.Mutex);
    video.ImageMsg = msg;
    videoEntity.GetComponent<ExpirableComponent>().restamp();
    // update pixel coordinates if image dimensions changed
    if (video.PixelBuffers.at(0).Width != video.ImageMsg->width
        || video.PixelBuffers.at(0).Height != video.ImageMsg->height) {
        auto &dynamic = videoEntity.GetComponent<DynamicDataComponent>();
        std::lock_guard<std::mutex> lock(*dynamic.Mutex);
        auto &renderable = videoEntity.GetComponent<RenderableElementComponent>();
        auto &mesh = renderable.Meshes.front();
        float newScalingX = float(video.ImageMsg->width) / float(video.WidthRaw);
        float newScalingY = float(video.ImageMsg->height) / float(video.HeightRaw);
        for (auto &vertex : mesh.Vertices) {
            vertex.TexCoord.x = newScalingX * (vertex.TexCoord.x - 1.0f) / video.ScalingX + 1.0f;
            vertex.TexCoord.y = newScalingY * (vertex.TexCoord.y - 1.0f) / video.ScalingY + 1.0f;
        }
        video.ScalingX = newScalingX;
        video.ScalingY = newScalingY;
        if (video.ImageMsg->encoding != "i420")
            ROS_WARN("%s: only i420 pixel format supported - received %s in Video::onImageReceived()",
                     ros::this_node::getName().c_str(), video.ImageMsg->encoding.c_str());
        video.PixelBuffers.at(0).Width = video.ImageMsg->width;
        video.PixelBuffers.at(0).Height = video.ImageMsg->height;
        video.PixelBuffers.at(1).Width = video.ImageMsg->width;
        video.PixelBuffers.at(1).Height = video.ImageMsg->height / 4;
        video.PixelBuffers.at(2).Width = video.ImageMsg->width;
        video.PixelBuffers.at(2).Height = video.ImageMsg->height / 4;
        dynamic.HasNewData = true;
    }
}

void Video::onProjectionReceived(const sensor_msgs::ImageConstPtr& msg, Entity &videoEntity) {
    auto &video = videoEntity.GetComponent<VideoComponent>();
    std::lock_guard<std::mutex> lock(*video.Mutex);
    video.ProjectionMsg = msg;
    video.PixelBuffers.at(3).Width = msg->width;
    video.PixelBuffers.at(3).Height = msg->height;
}

template<typename T>
void Video::init_mesh(Entity &ent, const T &camMdl, const geometry_msgs::TransformStamped tf2cam) {
    auto& video = ent.GetComponent<VideoComponent>();
    auto& renderable = ent.GetComponent<RenderableElementComponent>();
    auto& mesh = renderable.Meshes.front();
    mesh.Vertices.clear();
    mesh.Indices.clear();
    // init mesh depending on projection mode
    if (video.ProjectionMode == VideoComponent::ProjectionModeType::RECTANGULAR) {
        mesh.Vertices.push_back(Vertex(glm::vec3(0.0f,  -0.5f, 1.0f),
                                       glm::vec2(video.WidthRaw, 1.0f))); // top right
        mesh.Vertices.push_back(Vertex(glm::vec3(0.0f, -0.5f, 0.0f),
                                       glm::vec2(video.WidthRaw, video.HeightRaw))); // bottom right
        mesh.Vertices.push_back(Vertex(glm::vec3(0.0f, 0.5f, 0.0f),
                                       glm::vec2(1.0f, video.HeightRaw))); // bottom left
        mesh.Vertices.push_back(Vertex(glm::vec3(0.0f,  0.5f, 1.0f),
                                       glm::vec2(1.0f, 1.0f))); // top left
        mesh.Indices = { 0, 1, 3,  1, 2, 3 };
    } else {
        std::vector<std::vector<Vertex>> rowsOnSphere;
        size_t maxRowLength{0};
        if ((video.ProjectionMode == VideoComponent::ProjectionModeType::GROUND_PLANE)
            || (video.ProjectionMode == VideoComponent::ProjectionModeType::HALF_SPHERE_WITH_GROUND_PLANE)) {
            get_points_on_ground_plane(rowsOnSphere, maxRowLength, video, camMdl, tf2cam);
        }
        if ((video.ProjectionMode == VideoComponent::ProjectionModeType::SPHERE)
            || (video.ProjectionMode == VideoComponent::ProjectionModeType::HALF_SPHERE_WITH_GROUND_PLANE)
            || (video.ProjectionMode == VideoComponent::ProjectionModeType::ROBINSON)) {
            get_points_on_sphere(rowsOnSphere, maxRowLength, video, camMdl, tf2cam);
        }
        push_triangles_to_renderable(rowsOnSphere, maxRowLength, renderable);
    }
}

void Video::init_mesh_from_pinhole_model(Entity &ent, const PinholeModel &camMdl,
                                         const geometry_msgs::TransformStamped tf2cam) {
    init_mesh(ent, camMdl, tf2cam);
}

void Video::init_mesh_from_ocam_model(Entity &ent, const OcamModel &camMdl,
                                      const geometry_msgs::TransformStamped tf2cam) {
    init_mesh(ent, camMdl, tf2cam);
}

template <typename T>
void Video::get_points_on_ground_plane
    (std::vector<std::vector<Vertex>> &rowsOnSphere, size_t &maxRowLength,
     const VideoComponent &video, const T& camMdl, const geometry_msgs::TransformStamped &tf) {
    for (float myRad = video.GroundPlaneRadiusMin;
         myRad <= video.SphereRadius;
         myRad += SPHERE_MESH_INCREMENT) {
        std::vector<Vertex> verticesInRow;
        for (float lon = video.SphereLongitudeMax; lon >= video.SphereLongitudeMin; lon -= SPHERE_MESH_INCREMENT) {
            geometry_msgs::Pose ptOnSphere, poseOut;
            ptOnSphere.orientation.w = 1.0;
            ptOnSphere.position.x = myRad * std::cos(lon);
            ptOnSphere.position.y = myRad * std::sin(lon);
            ptOnSphere.position.z = 0.0;
            tf2::doTransform(ptOnSphere, poseOut, tf);
            if (!(poseOut.position.z > 0.0)) continue; // only z > can be projected

            // tf in pixel coords
            int x_px{0}, y_px{0};
            if (camMdl.point_on_image(poseOut.position, x_px, y_px)) {
                verticesInRow.push_back(Vertex(glm::vec3(ptOnSphere.position.x,
                                                         ptOnSphere.position.y,
                                                         ptOnSphere.position.z),
                                               glm::vec2(float(x_px), float(y_px))));
            }
        }
        if (!verticesInRow.empty()) {
            rowsOnSphere.push_back(verticesInRow);
            maxRowLength = std::max(maxRowLength, verticesInRow.size());
        }
    }
}

template <typename T>
void Video::get_points_on_sphere
    (std::vector<std::vector<Vertex>> &rowsOnSphere, size_t &maxRowLength,
     const VideoComponent &video, const T &vidParams, const geometry_msgs::TransformStamped &tf) {
    float latMin = video.SphereLatitudeMin;
    if (video.ProjectionMode == VideoComponent::ProjectionModeType::HALF_SPHERE_WITH_GROUND_PLANE)
        latMin = std::max(latMin, 0.0f);
    float latMax = video.SphereLatitudeMax;
    for (float lat = latMin; lat <= latMax; lat += SPHERE_MESH_INCREMENT) {
        std::vector<Vertex> verticesInRow;
        for (float lon = video.SphereLongitudeMax; lon >= video.SphereLongitudeMin; lon -= SPHERE_MESH_INCREMENT) {
            // tf point on sphere in camera coords
            geometry_msgs::Pose ptOnSphere, poseOut;
            ptOnSphere.orientation.w = 1.0;
            ptOnSphere.position.x = video.SphereRadius * std::cos(lat) * std::cos(lon);
            ptOnSphere.position.y = video.SphereRadius * std::cos(lat) * std::sin(lon);
            ptOnSphere.position.z = video.SphereRadius * std::sin(lat);
            tf2::doTransform(ptOnSphere, poseOut, tf);
            if (!(poseOut.position.z > 0.0)) continue; // only z > can be projected

            // tf in pixel coords
            int x_px{0}, y_px{0};
            if (vidParams.point_on_image(poseOut.position, x_px, y_px)) {
                if (video.ProjectionMode == VideoComponent::ProjectionModeType::ROBINSON) {
                    // equations following Savric et al.: A Polynomial Equation for the Natural Earth Projection
                    double A0{0.8507}, A1{0.9642}, A2{-0.1450};
                    double A3{-0.0013}, A4{-0.0104}, A5{-0.0129};
                    ptOnSphere.position.x = video.SphereRadius;
                    ptOnSphere.position.y = video.SphereRadius*lon*(A0 + A2*std::pow(lat, 2) + A4*std::pow(lat, 4));
                    ptOnSphere.position.z = video.SphereRadius*(A1*lat + A3*std::pow(lat, 3) + A5*std::pow(lat, 5));
                }
                verticesInRow.push_back(Vertex(glm::vec3(ptOnSphere.position.x,
                                                         ptOnSphere.position.y,
                                                         ptOnSphere.position.z),
                                               glm::vec2(float(x_px), float(y_px))));
            }
        }
        if (!verticesInRow.empty()) {
            rowsOnSphere.push_back(verticesInRow);
            maxRowLength = std::max(maxRowLength, verticesInRow.size());
        }
    }
}

void Video::push_triangles_to_renderable(std::vector<std::vector<Vertex>> &rowsOnSphere,
                                         const size_t maxRowLength,
                                         RenderableElementComponent &renderable) {
    // make all rows of equal length
    for (auto &row : rowsOnSphere) {
        for (size_t i = row.size(); i < maxRowLength; ++i) {
            row.push_back(row.back());
        }
    }

    // connect to triangles
    int ptCount{0};
    auto &mesh = renderable.Meshes.front();
    for (int i=1; i < rowsOnSphere.size(); ++i) {
        int a = ptCount++;
        mesh.Vertices.push_back(rowsOnSphere.at(i).at(0));
        int b = ptCount++;
        mesh.Vertices.push_back(rowsOnSphere.at(i-1).at(0));
        for (int j=1; j < rowsOnSphere.at(i).size(); ++j) {
            mesh.Vertices.push_back(rowsOnSphere.at(i).at(j));
            int c1 = ptCount++;
            mesh.Indices.push_back(a);
            mesh.Indices.push_back(b);
            mesh.Indices.push_back(c1);

            mesh.Vertices.push_back(rowsOnSphere.at(i-1).at(j));
            int c2 = ptCount++;
            mesh.Indices.push_back(b);
            mesh.Indices.push_back(c1);
            mesh.Indices.push_back(c2);
            a = c1;
            b = c2;
        }
    }
}

}; // namespace TodStandardEntities
