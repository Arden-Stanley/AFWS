#pragma once

#include <Arduino.h>
#include "AGVPinsDef.h"




enum mainState {
    AutoDrive,
    ManualDrive
};

// Do we need enum class or enum?
enum subState {
    START,
    INBETWEEN_ROWS,
    END_OF_ROW,
    TURNING,
    ALIGNING,
    ESTOP,
    COLLISION_AVOIDANCE
};

struct Config {
    detectionAngle = 180;


};

struct pointData {
    float x,
    float y
};

struct line {
  uint8_t A, B, C
};

struct Waypoint {
    float x,
    float y,
    uint16_t i
};


class stateMachine {
    public:
        MainState mState;
        SubState sState;
        pointData leftRow;
        pointData rightRow;
        Config set;
    private:



}
