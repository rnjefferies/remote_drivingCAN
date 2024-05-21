// Copyright 2020 TUMFTM

#pragma once
#include "Events/MouseEvent.h"
#include "geometry_msgs/Point.h"
#include <glad/glad.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "GL/glu.h"

class CursorPosition {
    public:
        static geometry_msgs::Point getRealWorldCoordinates(const MouseMovedEvent& mouseMovedEvent,
            const glm::mat4& modelViewMatrix, const glm::mat4& projectionMatrix);

    private:
        static void convertMat4ToGLdoubleArray(const glm::mat4& glmMat, GLdouble array[16]);
        static bool cursorHoveredOverObject(const GLfloat& depth);
        static GLint convertyPixelCoordFromGLFWToOpenGL(const unsigned int windowheight,
            const float yPosInPixelGLFW);
        static void printTmpMousePositionForDebugging(const geometry_msgs::Point& tmpClick);
};
