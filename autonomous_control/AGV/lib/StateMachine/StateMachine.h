#pragma once

#include <Arduino.h>
#include "AGVPinsDef.h"

enum class mainState {
    AutoDrive,
    ManualDrive
};

enum class subState {
    INBETWEEN_ROWS,
    END_OF_ROW,
    COLLISION_AVOIDANCE
};

class stateMachine {
    public:


    private:



}