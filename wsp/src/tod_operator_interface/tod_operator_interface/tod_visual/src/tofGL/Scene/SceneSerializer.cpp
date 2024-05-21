// Copyright 2020 TUMFTM based on Cherno's Hazel
#include "SceneSerializer.h"

bool Serialization::DeserializeEntities(const std::string &pkgPath, const std::string &vehicleID,
                                        std::vector<Entity> &entities) {
    std::string filepath = GetSceneFilePathFrom(pkgPath, vehicleID);
    if (!FileExists(filepath)) {
        // first blood
        SerializeEntities(pkgPath, vehicleID, entities);
        return false;
    }

    YAML::Node data = YAML::LoadFile(filepath);
    for (auto& entity : entities) {
        std::string tag = entity.GetComponent<TagComponent>().Tag;
        auto entityConfig = data[tag];
        if (entityConfig) {
            if (entity.HasComponent<TransformComponent>()) {
                auto tfConfig = entityConfig["TransformComponent"];
                if (tfConfig) {
                    auto& tc = entity.GetComponent<TransformComponent>();
                    tc.Translation = tfConfig["Translation"].as<glm::vec3>();
                    tc.Rotation = tfConfig["Rotation"].as<glm::vec3>();
                    tc.Scale = tfConfig["Scale"].as<glm::vec3>();
                }
            }

            // if (entity.HasComponent<RenderableElementComponent>()) {
            //     auto rcConfig = entityConfig["RenderableElementComponent"];
            //     if (rcConfig) {
            //         auto &rc = entity.GetComponent<RenderableElementComponent>();
            //         rc.StaticShow = rcConfig["StaticShow"].as<bool>();
            //         rc.RenderMode = rcConfig["RenderMode"].as<GLenum>();
            //         rc.LineWidth = rcConfig["LineWidth"].as<float>();
            //         rc.PointSize = rcConfig["PointSize"].as<float>();
            //     }
            // }

            if (entity.HasComponent<VideoComponent>()) {
                auto pcConfig = entityConfig["VideoComponent"];
                if (pcConfig) {
                    auto& pc = entity.GetComponent<VideoComponent>();
                    pc.ProjectionMode = VideoComponent::ProjectionModeType(
                        pcConfig["ProjectionMode"].as<int>());
                    pc.GroundPlaneRadiusMin = pcConfig["GroundPlaneRadiusMin"].as<float>();
                    pc.SphereRadius = pcConfig["SphereRadius"].as<float>();
                    pc.SphereLongitudeMin = pcConfig["SphereLongitudeMin"].as<float>();
                    pc.SphereLongitudeMax = pcConfig["SphereLongitudeMax"].as<float>();
                    pc.SphereLatitudeMin = pcConfig["SphereLatitudeMin"].as<float>();
                    pc.SphereLatitudeMax = pcConfig["SphereLatitudeMax"].as<float>();
                }
            }
        }
    }

    return true;
}

void Serialization::SerializeEntities(const std::string &pkgPath, const std::string &vehicleID,
                                      std::vector<Entity> &entities) {
    YAML::Emitter out;
    out << YAML::BeginMap; // File

    for (auto& entity : entities) {
        out << YAML::Key << entity.GetComponent<TagComponent>().Tag;
        out << YAML::BeginMap; // Entity
        if (entity.HasComponent<TransformComponent>()) {
            out << YAML::Key << "TransformComponent";
            out << YAML::BeginMap; // TransformComponent
            const auto& tc = entity.GetComponent<TransformComponent>();
            out << YAML::Key << "Translation" << YAML::Value << tc.Translation;
            out << YAML::Key << "Rotation" << YAML::Value << tc.Rotation;
            out << YAML::Key << "Scale" << YAML::Value << tc.Scale;
            out << YAML::EndMap; // TransformComponent
        }

        // if (entity.HasComponent<RenderableElementComponent>()) {
        //     out << YAML::Key << "RenderableElementComponent";
        //     out << YAML::BeginMap; // RenderableElementComponent
        //     const auto& rc = entity.GetComponent<RenderableElementComponent>();
        //     out << YAML::Key << "StaticShow" << YAML::Value << rc.StaticShow;
        //     out << YAML::Key << "RenderMode" << YAML::Value << rc.RenderMode;
        //     out << YAML::Key << "LineWidth" << YAML::Value << rc.LineWidth;
        //     out << YAML::Key << "PointSize" << YAML::Value << rc.PointSize;
        //     out << YAML::EndMap; // RenderableElementComponent
        // }

        if (entity.HasComponent<VideoComponent>()) {
            out << YAML::Key << "VideoComponent";
            out << YAML::BeginMap; // VideoComponent
            const auto& pc = entity.GetComponent<VideoComponent>();
            out << YAML::Key << "ProjectionMode" << YAML::Value << pc.ProjectionMode;
            out << YAML::Key << "GroundPlaneRadiusMin" << YAML::Value << pc.GroundPlaneRadiusMin;
            out << YAML::Key << "SphereRadius" << YAML::Value << pc.SphereRadius;
            out << YAML::Key << "SphereLongitudeMin" << YAML::Value << pc.SphereLongitudeMin;
            out << YAML::Key << "SphereLongitudeMax" << YAML::Value << pc.SphereLongitudeMax;
            out << YAML::Key << "SphereLatitudeMin" << YAML::Value << pc.SphereLatitudeMin;
            out << YAML::Key << "SphereLatitudeMax" << YAML::Value << pc.SphereLatitudeMax;
            out << YAML::EndMap; // VideoComponent
        }

        out << YAML::EndMap; // Entity
    }
    out << YAML::EndMap; // File

    std::string filepath = GetSceneFilePathFrom(pkgPath, vehicleID);
    std::ofstream fout(filepath);
    fout << out.c_str();
}
