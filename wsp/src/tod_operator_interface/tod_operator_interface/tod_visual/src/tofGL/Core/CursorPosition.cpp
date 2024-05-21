// Copyright 2020 TUMFTM
#include "CursorPosition.h"

geometry_msgs::Point CursorPosition::getRealWorldCoordinates(const MouseMovedEvent& mouseMovedEvent,
        const glm::mat4& modelViewMatrix, const glm::mat4& projectionMatrix) {
    geometry_msgs::Point tmpMousePosition;
    GLfloat depth;
    GLint yPosInOpenGLConvention = convertyPixelCoordFromGLFWToOpenGL(mouseMovedEvent.windowHeight,
            mouseMovedEvent.yPosInPixel);
    glReadPixels((GLint)mouseMovedEvent.xPosInPixel, yPosInOpenGLConvention,
            1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    if (cursorHoveredOverObject(depth)) {
        GLdouble arrayModelView[16] = {0.0};
        convertMat4ToGLdoubleArray(modelViewMatrix, arrayModelView);
        GLdouble arrayProjection[16] = {0.0};
        convertMat4ToGLdoubleArray(projectionMatrix, arrayProjection);
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        GLdouble x, y, z;
        gluUnProject(mouseMovedEvent.xPosInPixel, yPosInOpenGLConvention, depth,
                arrayModelView, arrayProjection, viewport, &x, &y, &z);
        tmpMousePosition.x = (double) x;
        tmpMousePosition.y = (double) y;
        tmpMousePosition.z = (double) z;
    }

    if (false) {
        printTmpMousePositionForDebugging(tmpMousePosition);
    }
    return tmpMousePosition;
}

GLint CursorPosition::convertyPixelCoordFromGLFWToOpenGL(
        const unsigned int windowHeight, const float yPosInPixelGLFW) {
    GLint yPosInOpenGLConvention = (GLint) (windowHeight - 1 - yPosInPixelGLFW);
    return yPosInOpenGLConvention;
}

bool CursorPosition::cursorHoveredOverObject(const GLfloat& depth) {
    return (depth > 0.0f && depth < 1.0f);
}

void CursorPosition::convertMat4ToGLdoubleArray(const glm::mat4& glmMat, GLdouble array[16]) {
    const float *pSource = (const float*)glm::value_ptr(glmMat);
    for (int i = 0; i < 16; ++i) {
        array[i] = pSource[i];
    }
}

void CursorPosition::printTmpMousePositionForDebugging(const geometry_msgs::Point& tmpMousePosition) {
    printf("x: %f y: %f y: %f\n", tmpMousePosition.x, tmpMousePosition.y, tmpMousePosition.z);
}
