// Copyright 2020 TUMFTM based on Cherno's Hazel
#pragma once
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>
#include "Scene.h"
#include "Components.h"
#include "Entity.h"

namespace Serialization {
bool DeserializeEntities(const std::string &pkgPath, const std::string &vehicleID,
                         std::vector<Entity> &entities);

void SerializeEntities(const std::string &pkgPath, const std::string &vehicleID,
                       std::vector<Entity> &entities);

static std::string GetSceneFilePathFrom(const std::string &pkgPath, const std::string& vehicleID) {
    return std::string(pkgPath + "/yaml/" + vehicleID + ".yaml");
}

static bool FileExists(const std::string &filepath) {
    return std::ifstream(filepath).good();
}
}; // namespace Serialization


namespace YAML {
static Emitter& operator<<(YAML::Emitter &out, const glm::vec3 &v) {
    out << YAML::Flow;
    out << YAML::BeginSeq << v.x << v.y << v.z << YAML::EndSeq;
    return out;
}

static Emitter& operator<<(YAML::Emitter &out, const glm::vec4 &v) {
    out << YAML::Flow;
    out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
    return out;
}

template<>
struct convert<glm::vec3> {
    static Node encode(const glm::vec3 &rhs) {
        Node node;
        node.push_back(rhs.x);
        node.push_back(rhs.y);
        node.push_back(rhs.z);
        node.SetStyle(YAML::EmitterStyle::Flow);
        return node;
    }

    static bool decode(const Node &node, glm::vec3 &rhs) {
        if (!node.IsSequence() || node.size() != 3)
            return false;

        rhs.x = node[0].as<float>();
        rhs.y = node[1].as<float>();
        rhs.z = node[2].as<float>();
        return true;
    }
};

template<>
struct convert<glm::vec4> {
    static Node encode(const glm::vec4 &rhs) {
        Node node;
        node.push_back(rhs.x);
        node.push_back(rhs.y);
        node.push_back(rhs.z);
        node.push_back(rhs.w);
        node.SetStyle(EmitterStyle::Flow);
        return node;
    }

    static bool decode(const Node &node, glm::vec4 &rhs) {
        if (!node.IsSequence() || node.size() != 4)
            return false;

        rhs.x = node[0].as<float>();
        rhs.y = node[1].as<float>();
        rhs.z = node[2].as<float>();
        rhs.w = node[3].as<float>();
        return true;
    }
};
}; // namespace YAML
