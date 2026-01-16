#include "StateMachine.h"
#include "AGVPins.h"

pins p;

void stop(){
  analogWrite(p.PWM1, 0);
  analogWrite(p.PWM2, 0);
}

void inbetween_rows(){
  delay(2000);
  digitalWrite(p.INB1, HIGH);
  digitalWrite(p.INB2, HIGH);
  analogWrite(p.PWM1, 70);
  analogWrite(p.PWM2, 70);

}

void collision_avoidance(){

}

void leftCorrect(){

}

void rightCorrected(){

}
