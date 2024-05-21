// Copyright 2021 TUMFTM
#include "ShaderSystem.h"
#include "Scene/Components.h"
#include <fstream>
#include <iostream>
#include <sstream>

unsigned int ShaderSystem::createShaderProgram(const char* vertexPath, const char* fragmentPath) {
    const unsigned int programID = glCreateProgram();
    attachShaderToProgram(programID, vertexPath, GL_VERTEX_SHADER);
    attachShaderToProgram(programID, fragmentPath, GL_FRAGMENT_SHADER);
    glLinkProgram(programID);
    checkShaderCompileErrors(programID, "PROGRAM");
    return programID;
}

void ShaderSystem::useShaderProgram(const unsigned int programID) {
    glUseProgram(programID);
}

void ShaderSystem::attachShaderToProgram(const unsigned int programID, const char* shaderPath, GLenum shaderType) {
    // read source code from file
    std::ifstream shaderFile;
    shaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    std::string shaderCode;
    try {
        shaderFile.open(shaderPath);
        std::stringstream shaderStream;
        shaderStream << shaderFile.rdbuf();
        shaderFile.close();
        shaderCode = shaderStream.str();
    } catch(std::ifstream::failure e) {
        std::cout << "ERROR: could not read shader" << std::endl;
    }
    const char* source = shaderCode.c_str();

    // create, compile, attach and delete shader
    unsigned int shader = glCreateShader(shaderType);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    checkShaderCompileErrors(shader, "SHADER");
    glAttachShader(programID, shader);
    glDeleteShader(shader);
}

void ShaderSystem::setShaderProgramBool(const unsigned int programID, const std::string &name, bool value) {
    ShaderSystem::useShaderProgram(programID);
    glUniform1i(glGetUniformLocation(programID, name.c_str()), (int)value);
}

void ShaderSystem::setShaderProgramInt(const unsigned int programID, const std::string &name, int value) {
    ShaderSystem::useShaderProgram(programID);
    glUniform1i(glGetUniformLocation(programID, name.c_str()), value);
}

void ShaderSystem::setShaderProgramFloat(const unsigned int programID, const std::string &name, float value) {
    ShaderSystem::useShaderProgram(programID);
    glUniform1f(glGetUniformLocation(programID, name.c_str()), value);
}

void ShaderSystem::setShaderProgramVec3(const unsigned int programID, const std::string &name, const glm::vec3 &value) {
    ShaderSystem::useShaderProgram(programID);
    glUniform3f(glGetUniformLocation(programID, name.c_str()), value.x, value.y, value.z);
}

void ShaderSystem::setShaderProgramMat4(const unsigned int programID, const std::string &name, const glm::mat4 &value) {
    ShaderSystem::useShaderProgram(programID);
    glUniformMatrix4fv(glGetUniformLocation(programID, name.c_str()), 1, GL_FALSE, glm::value_ptr(value));
}

void ShaderSystem::checkShaderCompileErrors(unsigned int programID, std::string type) {
    int success;
    char infoLog[1024];
    if (type != "PROGRAM") {
        glGetShaderiv(programID, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(programID, 1024, NULL, infoLog);
            std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n"
                      << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
        }
    } else {
        glGetProgramiv(programID, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(programID, 1024, NULL, infoLog);
            std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n"
                      << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
        }
    }
}
