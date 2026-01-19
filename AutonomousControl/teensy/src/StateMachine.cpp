#include "../include/StateMachine.h"
#include "../include/AGVPins.h"

pins p;

void StateMachine::stop(){
  analogWrite(p.PWM1, 0);
  analogWrite(p.PWM2, 0);
}

/* PID Description:
  A PID (Proportion, Integral, Derivative) is a closed loop (feedback) system.
  Essentially, it takes error over time and uses it as a way to send power to the wheels when needed.
  We use this for wall-following.
  Formula = u(t) = Kp*e(t) + Ki*integral(e(tau)dtau) + Kd*de(t)/dt

  Here, sideDifference is the absolute value of the right subtracted by the left.
  We want to keep it at 0, which means that both sides are even, which is what 'desiredState' is.
*/
void StateMachine::PID(float sideDifference){
  float headingCorrection = ransac.bestLine.a * 500.0f;

  float error = (desiredState - sideDifference) + headingCorrection;

//Kp*e(t)
  double proportional = Kp * error;

//Ki*integral(e(tau)dtau)
  kiTotal += error;
  kiTotal = constrain(kiTotal, -100, 100);
  double integral = Ki * kiTotal;

//Kd*de(t)/dt
  float derivative = Kd * (error - prevError);
  prevError = error;

// u(t) = Kp*e(t) + Ki*integral(e(tau)dtau) + Kd*de(t)/dt
  pidResult = proportional + integral + derivative;
// Constrains over-all result
  pidResult = constrain(pidResult, -100, 100);
}

void StateMachine::inbetween_rows(float sideDifference){
  digitalWrite(p.INB1, HIGH);
  digitalWrite(p.INB2, HIGH);
  analogWrite(p.PWM1, pwmSpeed);
  analogWrite(p.PWM2, pwmSpeed);

  PID(sideDifference);

// Multiplies pidResult if there is a large difference in distance.
// Helps the car turn more aggresively if it is too far away.
  if(abs(desiredState - sideDifference) > 200){
    pidResult *= 1.5;
  }

// Limits the steering so it doesn't over-steer
  int steeringLimit = pwmSpeed * 0.55;
  pidResult = constrain(pidResult, -steeringLimit, steeringLimit);

  int leftSpeed = pwmSpeed + pidResult;
  int rightSpeed = pwmSpeed - pidResult;

// Constrains speed between 0 to 255, which is the limit for the motors
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(p.PWM1, leftSpeed);
  analogWrite(p.PWM2, rightSpeed);
}

//void StateMachine::end_of_row(){}

//void StateMachine::detected_weed(){}
