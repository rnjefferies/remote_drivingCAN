// Copyright 2021 TUMFTM
#pragma once
#include <glad/glad.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <string>

class ShaderSystem {
public:
    ~ShaderSystem();
    static unsigned int createShaderProgram(const char* vertexPath, const char* fragmentPath);
    static void useShaderProgram(const unsigned int programID);
    static void attachShaderToProgram(const unsigned int programID, const char* shaderPath, GLenum shaderType);
    static void setShaderProgramBool(const unsigned int programID, const std::string &name, bool value);
    static void setShaderProgramInt(const unsigned int programID, const std::string &name, int value);
    static void setShaderProgramFloat(const unsigned int programID, const std::string &name, float value);
    static void setShaderProgramVec3(const unsigned int programID, const std::string &name, const glm::vec3 &value);
    static void setShaderProgramMat4(const unsigned int programID, const std::string &name, const glm::mat4 &value);

private:
    ShaderSystem();
    static void checkShaderCompileErrors(unsigned int programID, std::string type);
};
