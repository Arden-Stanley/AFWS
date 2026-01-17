#ifndef AGVPINS_H
#define AGVPINS_H

#include <Arduino.h>

class pins {
  public:
    const int PWM1 = 9;
    const int PWM2 = 6;
    const int INA1 = 7;
    const int INA2 = 4;
    const int INB1 = 8;
    const int INB2 = 5;
};

#endif