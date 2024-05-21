// Copyright 2020 TUMFTM
#pragma once
#include "Scene/Entity.h"
#include "Scene/Components.h"
#include "Systems/Renderer.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "stb_image/stb_image.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <memory>
namespace tofGL {
class ModelLoader {
public:
    ModelLoader(std::shared_ptr<Scene> activeScene, const std::string &modelPath);
    Entity loadModel(const std::string &modelName, Entity parent, const glm::vec3 &scaling, const glm::vec3 &rotation,
        const std::string &packagePath);

private:
    std::vector<Texture> _textures;
    std::vector<std::string> _texturePaths;
    std::vector<Mesh> _meshes;
    std::shared_ptr<Scene> _activeScene;
    std::string _modelPath;
    std::string _packagePath;
    std::string _modelName;
    Texture _emptyTexture;
    Texture textureFromFile(const char *path, const int shader);
    void processNode(aiNode *node, const aiScene *scene, const int shader);
    Mesh processMesh(aiMesh *mesh, const aiScene *scene, const int shader);
    std::vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type, const int shader);
};
}; // namespace tofGL
