// Copyright 2020 TUMFTM
#pragma once

namespace joystick {
enum ButtonPos {
    INDICATOR_LEFT  = 0,
    INDICATOR_RIGHT = 1,
    FLASHLIGHT      = 2,
    FRONTLIGHT      = 3,
    HONK            = 4,
    INCREASE_SPEED  = 5,
    DECREASE_SPEED  = 6,
    INCREASE_GEAR   = 7,
    DECREASE_GEAR   = 8
};

enum AxesPos {
    STEERING = 0,
    THROTTLE = 1,
    BRAKE = 2
};
}; // namespace joystick
