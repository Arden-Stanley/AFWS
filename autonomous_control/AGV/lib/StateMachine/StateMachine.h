#pragma once

#include <Arduino.h>
#include "AGVPins.h"

enum class MainState {
    AutoDrive,
    ManualDrive
};


enum class SubState {
    INBETWEEN_ROWS,
    END_OF_ROW,
    COLLISION_AVOIDANCE
};

