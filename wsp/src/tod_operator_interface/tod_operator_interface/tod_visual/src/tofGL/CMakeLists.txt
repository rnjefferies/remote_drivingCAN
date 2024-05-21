if (POLICY CMP0072)#supress warning for find_package(OpenGL)
        cmake_policy (SET CMP0072 OLD)
endif()
find_package(assimp REQUIRED)
find_package(GLM REQUIRED)
find_package(glfw3 3.2 REQUIRED)
find_package(Freetype REQUIRED)
find_package(OpenVR REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenGL REQUIRED)

add_subdirectory(libs/glad)
add_subdirectory(libs/entt)
add_subdirectory(libs/stb_image)

set(ogl_sources
    Core/Window.cpp
    Core/Application.cpp
    Core/CameraController.cpp
    Core/DataContainer.h
    Core/ModelLoader.cpp
    Core/CursorPosition.cpp
    Core/GraphicsContext.cpp
    Core/RenderCommand.cpp
    Core/RendererAPI.cpp
    Core/RosInterface.cpp

    Window/GLFWWindow.cpp
    
    Systems/CameraSystem.cpp
    Systems/Renderer.cpp
    Systems/TransformSystem.cpp
    Systems/ShaderSystem.cpp
    Systems/VRSystem.cpp

    OpenGL/OpenGLContext.cpp
    OpenGL/OpenGLRendererAPI.cpp

    Scene/Components.h
    Scene/Entity.h
    Scene/Scene.cpp
    Scene/SceneSerializer.cpp

    Events/Event.cpp
    Events/ApplicationEvent.cpp
    Events/MouseEvent.cpp
    Events/KeyEvent.cpp
)

add_library(tofGL ${ogl_sources})
target_include_directories(tofGL 
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}   #Todo: make only relevant Include folders public -> #include "tofGL/KoSys.h"
        ${YAML_CPP_INCLUDE_DIR}
        ${FREETYPE_INCLUDE_DIRS}
    PRIVATE
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(tofGL 
    PRIVATE
        OpenGL::GLU
        glfw
        OpenVR::OpenVR
        assimp
        ${catkin_LIBRARIES}
    PUBLIC
        GLM::GLM
        ${YAML_CPP_LIBRARIES}
        ${FREETYPE_LIBRARIES}
        glad
        entt
        stb_image
)
target_compile_features(tofGL PUBLIC cxx_std_17)

install(TARGETS tofGL
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
