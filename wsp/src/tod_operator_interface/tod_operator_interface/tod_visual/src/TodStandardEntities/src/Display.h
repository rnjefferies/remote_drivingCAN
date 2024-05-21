// Copyright 2020 Feiler
#pragma once
#include "Scene/Components.h"
#include "Scene/Entity.h"
#include "Scene/Scene.h"
#include "memory.h"
#include "tod_msgs/VehicleData.h"
#include "tod_msgs/PrimaryControlCmd.h"
#include "tod_msgs/SecondaryControlCmd.h"
#include "tod_msgs/VehicleEnums.h"
#include "Systems/ShaderSystem.h"
#include "Systems/Renderer.h"
#include <ft2build.h>
#include FT_FREETYPE_H
#include <memory>
#include <vector>
#include <utility>
#include <string>

namespace TodStandardEntities {

class Display {
public:
    static Entity create(std::shared_ptr<Scene> scene,
                         std::string name, Entity parent, const std::string& packagePath);
    static void onSpeedUpdate(const tod_msgs::VehicleDataConstPtr& msg, Entity &entity);
    static void onDesiredSpeedUpdate(const tod_msgs::PrimaryControlCmdConstPtr& msg, Entity &entity);
    static void onGearUpdate(const tod_msgs::VehicleDataConstPtr& msg, Entity &entity);
    static void onDesiredGearUpdate(const tod_msgs::SecondaryControlCmdConstPtr& msg, Entity &entity);

private:
    Display() = default;
    static void GenerateCharactersWithTextureForDisplay(
        const std::string &pathToTTF, CharacterMapComponent &charMap, RenderableElementComponent &renderable);
    static std::vector<RenderableElementComponent> createCharacterRenderables(
        const int maxExpectedCharacters, const std::string& relativeVertexShaderPath,
        const std::string& relativeFragmentShaderPath, const std::string& packagePath);
    static void clearVerticesAndTextures(RenderableElementComponent& renderable);
    static void updateVertices(Mesh& currentMesh, const float tmpAdvance,
                               const Character& currentChar, const float& scale,
                               const float& y, const float& ratioPixelPerMeter);
    static void updateAdvance(float& tmpAdvance, const Character& currentChar,
                              const float& scale);
    static void writeTextIntoRenderable(const std::string& istGeschwindigkeit,
                                        RenderableElementComponent& renderable,
                                        const CharacterMapComponent& characterMap);
    static std::string floatToIntString(const float number);
    static std::string eGearPositionToString(const int gearPosition);
};

}; // namespace TodStandardEntities
