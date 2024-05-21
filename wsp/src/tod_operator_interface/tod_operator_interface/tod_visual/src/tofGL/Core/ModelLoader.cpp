// Copyright 2020 TUMFTM
#include "Core/ModelLoader.h"

namespace tofGL {

ModelLoader::ModelLoader(std::shared_ptr<Scene> activeScene,
        const std::string &modelPath) : _activeScene(activeScene), _modelPath(modelPath) {
}

Entity ModelLoader::loadModel(const std::string& modelName, Entity parent,
        const glm::vec3 & scaling, const glm::vec3& rotation, const std::string &packagePath) {
    _modelName = modelName;
    _meshes.clear();

    unsigned int modelShader = ShaderSystem::createShaderProgram(
        (packagePath + "/resources/OpenGL/shaders/model.vert").c_str(),
        (packagePath + "/resources/OpenGL/shaders/model.frag").c_str());

    GLubyte data[] = { 255, 255, 255, 255 };
    _emptyTexture = Texture(1, 1, "texture_diffuse", GL_TEXTURE_2D, GL_RGB);
    Renderer::GenerateTexture(_emptyTexture, data, modelShader, 0);

    //Get File Path of Model.obj
    std::string filePath = _modelPath + _modelName + "/" + _modelName + ".obj";

    Entity model = _activeScene->CreateEntity(_modelName);
    model.GetComponent<TransformComponent>().setParent(parent);
    model.GetComponent<TransformComponent>().setScale(scaling);
    model.GetComponent<TransformComponent>().setRotation(rotation);

    // Import aiScene
    Assimp::Importer import;
    const aiScene *scene = import.ReadFile(filePath, aiProcess_Triangulate | aiProcess_FlipUVs
        | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices);
    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        std::cout << "ERROR::ASSIMP::" << import.GetErrorString() << std::endl;
    } else {// Process Nodes Recursive
        processNode(scene->mRootNode, scene, modelShader);
        model.AddComponent<RenderableElementComponent>(modelShader, _meshes);
    }
    return model;
}

void ModelLoader::processNode(aiNode *node, const aiScene *scene, const int shader) {
    // process all the node's meshes (if any)
    for (unsigned int i = 0; i < node->mNumMeshes; i++) {
        aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
        _meshes.push_back(processMesh(mesh, scene, shader));
    }
    // then do the same for each of its children
    for (unsigned int i = 0; i < node->mNumChildren; i++) {
        processNode(node->mChildren[i], scene, shader);
    }
}

Mesh ModelLoader::processMesh(aiMesh *mesh, const aiScene *scene, const int shader) {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<Texture> textures;
    aiColor4D *materialColor = new aiColor4D;
    aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
    textures = loadMaterialTextures(material, aiTextureType_DIFFUSE, shader);
    aiGetMaterialColor(material, AI_MATKEY_COLOR_DIFFUSE, materialColor);
    for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
        Vertex vertex;
        vertex.Position.x = mesh->mVertices[i].x;
        vertex.Position.y = mesh->mVertices[i].y;
        vertex.Position.z = mesh->mVertices[i].z;
        vertex.TexCoord = (mesh->mTextureCoords[0])
            ? glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y)
            : glm::vec2(0.0f, 0.0f);
        vertex.TexColor = glm::vec3(materialColor->r, materialColor->g, materialColor->b);
        vertices.emplace_back(vertex);
    }
    for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
        aiFace face = mesh->mFaces[i];
        for (unsigned int j = 0; j < face.mNumIndices; j++)
            indices.emplace_back(face.mIndices[j]);
    }
    return Mesh(vertices, indices, textures);
}

std::vector<Texture> ModelLoader::loadMaterialTextures(aiMaterial *mat, aiTextureType type, const int shader) {
    std::vector<Texture> textures;
    unsigned int i = 0;
    do {
        aiString str;
        mat->GetTexture(type, i, &str);

        if (mat->GetTextureCount(type) == 0) {
            textures.emplace_back(_emptyTexture);
            return textures;
        }

        bool skip = false;
        for (unsigned int j = 0; j < _textures.size(); j++) {
            if (std::strcmp(_texturePaths.at(j).data(), str.C_Str()) == 0) {
                textures.emplace_back(Texture());
                textures.back().Id = _textures[j].Id;
                textures.back().Type = GL_TEXTURE_2D;
                skip = true;
                break;
            }
        }
        if (!skip) { // if texture hasn't been loaded already, load it
            textures.emplace_back(textureFromFile(str.C_Str(), shader));
            _texturePaths.emplace_back(std::string(str.C_Str()));
            _textures.emplace_back(textures.back()); // add to loaded textures
        }
        i++;
    } while (i < mat->GetTextureCount(type));
    return textures;
}

Texture ModelLoader::textureFromFile(const char *textureName, const int shader) {
    std::string filename = std::string(textureName);
    filename = _modelPath + _modelName + "/" + filename;
    int width, height, nrComponents;
    unsigned char *data = stbi_load(filename.c_str(), &width, &height, &nrComponents, 0);
    if (!data) {
        ROS_ERROR_STREAM("In ModelLoader::textureFromFile(): Texture failed to load at path: "
            << filename);
        return Texture();
    }
    GLenum format{GL_NONE};
    if (nrComponents == 1) format = GL_RED;
    else if (nrComponents == 3) format = GL_RGB;
    else if (nrComponents == 4) format = GL_RGBA;
    else ROS_WARN("In ModelLoader::textureFromFile(): format not initialized");
    Texture texture(width, height, "texture_diffuse", GL_TEXTURE_2D, format);
    Renderer::GenerateTexture(texture, data, shader, 0);
    stbi_image_free(data);
    return texture;
}
};  // namespace tofGL
