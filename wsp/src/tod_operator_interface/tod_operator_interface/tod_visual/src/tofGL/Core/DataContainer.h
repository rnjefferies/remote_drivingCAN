// Copyright 2020 TUMFTM
#pragma once
#define GLM_ENABLE_EXPERIMENTAL
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <ros/ros.h>
#include <glad/glad.h>

struct Vertex {
    glm::vec3 Position;
    glm::vec2 TexCoord;
    glm::vec3 TexColor;

    Vertex(const glm::vec3 &position = glm::vec3(0.0f, 0.0f, 0.0f),
           const glm::vec2 &texCoord = glm::vec2(0.0f, 0.0f),
           const glm::vec3 &texColor = glm::vec3(0.0f, 0.0f, 0.0f))
        : Position(position), TexCoord(texCoord), TexColor(texColor) {}
};

struct Texture {
    unsigned int Id{0}, Width{0}, Height{0};
    std::string Name{""};
    GLenum Type{GL_NONE};
    GLenum Format{GL_RED};

    Texture(const unsigned int width = 0, const unsigned int height = 0,
            const std::string &name = "", const GLenum type = GL_NONE, const GLenum format = GL_RED)
        : Width{width}, Height{height}, Name{name}, Type{type}, Format{format} {}
};

struct Buffer {
    GLuint Id{0};
    GLenum Target, Usage;

    Buffer(const GLenum target, const GLenum usage) : Target{target}, Usage{usage} {}
};

struct VertexArray {
    GLuint Id{0};
};

struct Mesh {
    std::vector<Vertex> Vertices;
    std::vector<unsigned int> Indices;
    std::vector<Texture> Textures;
    VertexArray VAO;
    Buffer VertexBuffer, IndexBuffer;

    Mesh(std::vector<Vertex> vertices,
         std::vector<unsigned int> indices = std::vector<unsigned int>(),
         std::vector<Texture> textures = std::vector<Texture>())
        : Vertices(vertices), Indices(indices), Textures(textures),
        VertexBuffer(GL_ARRAY_BUFFER, GL_STATIC_DRAW),
        IndexBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW) {
        if (vertices.empty()) {
            ROS_WARN("%s: Mesh(vertices) - empty vertices - filling up",
                     ros::this_node::getName().c_str());
            vertices = VectorOfThreeZeroVertices();
        }
    }

    static Mesh nonEmptyMesh() {
        Mesh mesh(Mesh::VectorOfThreeZeroVertices());
        return mesh;
    }

    static std::vector<Vertex> VectorOfThreeZeroVertices() {
        std::vector<Vertex> vertices;
        for (int i=0; i < 3; ++i) vertices.emplace_back(Vertex(glm::vec3(0.0f, 0.0f, 0.0f)));
        return vertices;
    }
};

struct Character {
    Texture Tex;
    glm::ivec2 Size;
    glm::ivec2 Bearing;
    unsigned int Advance;
};
