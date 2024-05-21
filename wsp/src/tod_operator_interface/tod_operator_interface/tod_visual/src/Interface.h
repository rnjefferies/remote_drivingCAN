// Copyright 2021 TUMFTM
#pragma once
static const float PI{3.14159f};
#include <memory>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <limits>
#include <future>
#include <map>

#include "glad/glad.h"
#include "GLFW/glfw3.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/matrix_transform.hpp>
#include <glm/glm.hpp>

#include "Core/Application.h"
#include "Core/CursorPosition.h"
#include "Core/ModelLoader.h"
#include "Scene/Components.h"
#include "Scene/Entity.h"
#include "Scene/Scene.h"
#include "Scene/SceneSerializer.h"
#include "tod_core/VehicleParameters.h"
#include "tod_core/CameraParameters.h"
#include "tod_core/LidarParameters.h"
#include "TodStandardEntities/TodStandardEntities.h"
#include "tod_core/TransformParameters.h"

class Interface : public Application {
public:
    Interface(int argc, char **argv);

private:
    std::unique_ptr<tod_core::VehicleParameters> _vehParams;
    std::unique_ptr<tod_core::CameraParameters> _camParams;
    std::unique_ptr<tod_core::LidarParameters> _lidarParams;
    std::unique_ptr<tod_core::TransformParameters> _transformParams;

    void CreateScene();
    void CreateCosysEntities(std::map<std::string, Entity> &coordinateSystems);
    void CreateDisplayEntities(const std::map<std::string, Entity> &coordinateSystems);
    void CreateVehicleModelEntities(const std::map<std::string, Entity> &coordinateSystems);
    void CreateGridAndFloorEntities(const std::map<std::string, Entity> &coordinateSystems);
    void CreateCameraFramebufferAndTopViewEntities(const std::map<std::string, Entity> &coordinateSystems);
    void CreateVideoEntities(const std::map<std::string, Entity> &coordinateSystems);
    void CreateLaneEntities(const std::map<std::string, Entity> &coordinateSystems);
    void CreateLaserScanEntities(const std::map<std::string, Entity> &coordinateSystems);
    bool DeserializeEntityData();
    void CreateVideoMeshes(const bool couldDeserialize);
    void CreateSafeCorridorControlEntities(const std::map<std::string, Entity> &coordinateSystems);
    void CreateSharedControlEntities(const std::map<std::string, Entity> &coordinateSystems);
};
