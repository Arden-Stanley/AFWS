#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "RANSAC.h"

class StateMachine {
  public:
    enum AGVSTATE { STOP, INBETWEEN_ROWS, END_OF_ROW };
    AGVSTATE STATE;
    //Cases
    void stop();
    void inbetween_rows(float sideDifference);
    void end_of_row();
    //Proportional Integral Derivative
    void PID(float sideDifference);

  private:
  //Tuning Variables
    const float Kp = 2;
    const float Ki = .5;
    const float Kd = 1;
  //desiredState at 0 means the difference between the distances from both walls is 0, aka they're even distance apart.
    const float desiredState = 0;
  //Errors
    float prevError = 0;
    float kiTotal = 0;
  //Used in inbetween_rows.
    int pwmSpeed = 70;
    float pidResult = 0;
};

#endif