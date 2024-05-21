// Copyright 2020 TUMFTM
#pragma once
// used in SecondaryControlCmd and VehicleData messages

enum eIndicator {
    INDICATOR_OFF = 0,
    INDICATOR_LEFT = 1,
    INDICATOR_RIGHT = 2,
    INDICATOR_BOTH = 3
};

enum eGearPosition {
    GEARPOSITION_PARK = 0,
    GEARPOSITION_REVERSE = 3,
    GEARPOSITION_NEUTRAL = 2,
    GEARPOSITION_DRIVE = 1,
    GEARPOSITION_SPORT = 4,
    GEARPOSITION_HAUL = 5
};

enum eHonk {
    HONK_OFF = 0,
    HONK_ON = 1
};

enum eWiper {
    WIPER_OFF = 0,
    WIPER_ON = 1,
    WIPER_INTERVAL = 2
};

enum eHeadLight {
    HEADLIGHT_OFF = 0,
    HEADLIGHT_ON = 1
};

enum eFlashLight {
    FLASHLIGHT_OFF = 0,
    FLASHLIGHT_ON = 1
};
