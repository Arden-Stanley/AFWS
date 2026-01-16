#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "RANSAC.h"
#include <Arduino.h>

class StateMachine {
  public:
  enum AGVSTATE { STOP, INBETWEEN_ROWS, COLLISION_AVOIDANCE, LEFT_CORRECT, RIGHT_CORRECT };
  
  AGVSTATE STATE;

  //Cases
  void stop();
  void inbetween_rows(int PWM);
  void collision_avoidance();
  void leftCorrect();
  void rightCorrect();
};

#endif