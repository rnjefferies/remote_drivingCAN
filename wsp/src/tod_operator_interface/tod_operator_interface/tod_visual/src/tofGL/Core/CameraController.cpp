// Copyright 2020 TUMFTM
#include "Core/CameraController.h"

void CameraController::onEvent(Event& e, CameraComponent& camera, float delta_time) {
   EventDispatcher dispatcher(e);

   if (e.GetEventType() == EventType::MouseScrolled) {
      MouseScrolledEvent mouse = static_cast<MouseScrolledEvent&>(e);
      if (mouse.GetYOffset() > 0) { moveCamera(delta_time, Z_POS_INCREASE, camera); }
      if (mouse.GetYOffset() < 0) { moveCamera(delta_time, Z_POS_DECREASE, camera); }
   }
   if (e.GetEventType() == EventType::KeyPressed) {
      KeyPressedEvent key = static_cast<KeyPressedEvent&>(e);
      switch (key.GetKeyCode()) {
      case KeyCode::Up :
         moveCamera(delta_time, RADIUS_DECREASE, camera);
         break;
      case KeyCode::Down :
         moveCamera(delta_time, RADIUS_INCREASE, camera);
         break;
      case KeyCode::Right :
         moveCamera(delta_time, YAW_DECREASE, camera);
         break;
      case KeyCode::Left :
         moveCamera(delta_time, YAW_INCREASE, camera);
         break;
      case KeyCode::PageUp :
         moveCamera(delta_time, Z_LOOKAT_INCREASE, camera);
         break;
      case KeyCode::PageDown :
         moveCamera(delta_time, Z_LOOKAT_DECREASE, camera);
         break;
      default:
         break;
      }
   }
}

void CameraController::moveCamera(float deltaTime, CameraMovement direction, CameraComponent& camera) {
    float cameraSpeed = 10 * deltaTime;
    switch (direction) {
        case YAW_INCREASE:
            camera.Yaw += cameraSpeed*10;
            break;
        case YAW_DECREASE:
            camera.Yaw -= cameraSpeed*10;
            break;
        case RADIUS_INCREASE:
            camera.Radius += cameraSpeed;
            break;
        case RADIUS_DECREASE:
            camera.Radius -= cameraSpeed;
            break;
        case Z_POS_INCREASE:
            camera.Position.z += cameraSpeed;
            break;
        case Z_POS_DECREASE:
            camera.Position.z -= cameraSpeed;
            break;
        case Z_LOOKAT_INCREASE:
            camera.LookAt.z += cameraSpeed;
            break;
        case Z_LOOKAT_DECREASE:
            camera.LookAt.z -= cameraSpeed;
            break;
        default:
            break;
    }
}
