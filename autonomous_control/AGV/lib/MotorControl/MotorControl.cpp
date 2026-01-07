#include "MotorControl.h"
#include <arduino.h>
#include "PinDefs.h"

void MotorControl::Forward(uint8_t speed) {
    analogWrite(LPWM, speed);
    analogWrite(RPWM, speed);
    digitalWrite(LINA, HIGH);
    digitalWrite(LINB, LOW);
    digitalWrite(RINA, HIGH);
    digitalWrite(RINB, LOW);
}

void MotorControl::Reverse(uint8_t speed) {
    analogWrite(LPWM, speed);
    analogWrite(RPWM, speed);
    digitalWrite(LINA, LOW);
    digitalWrite(LINB, HIGH);
    digitalWrite(RINA, LOW);
    digitalWrite(RINB, HIGH);
}

void MotorControl::Stop() {
    analogWrite(LPWM, 0);
    analogWrite(RPWM, 0);
    digitalWrite(LINA, HIGH);
    digitalWrite(LINB, HIGH);
    digitalWrite(RINA, HIGH);
    digitalWrite(RINB, HIGH);
}

void MotorControl::LeftTurn(uint8_t speed, int8_t intensity) {
    uint8_t leftPWM = clamp(speed - intensity, 0, 255);
    uint8_t rightPWM = clamp(speed + intensity, 0, 255);
    analogWrite(LPWM, leftPWM);
    analogWrite(RPWM, rightPWM);
    digitalWrite(LINA, HIGH);
    digitalWrite(LINB, LOW);
    digitalWrite(RINA, HIGH);
    digitalWrite(RINB, LOW);
}

void MotorControl::RightTurn(uint8_t speed, int8_t intensity) {
    uint8_t leftPWM = clamp(speed + intensity, 0, 255);
    uint8_t rightPWM = clamp(speed - intensity, 0, 255);
    analogWrite(LPWM, leftPWM);
    analogWrite(RPWM, rightPWM);
    digitalWrite(LINA, HIGH);
    digitalWrite(LINB, LOW);
    digitalWrite(RINA, HIGH);
    digitalWrite(RINB, LOW);
}

void MotorControl::AutoForward(uint8_t speed, int8_t turn) {
    int8_t left = speed + turn;
    int8_t right = speed - turn;
    left = clamp(left, 0, 255);
    right = clamp(right, 0, 255);
    analogWrite(LPWM, left);
    analogWrite(RPWM, right);
    digitalWrite(LINA, HIGH);
    digitalWrite(LINB, LOW);
    digitalWrite(RINA, HIGH);
    digitalWrite(RINB, LOW);
}

void MotorControl::AutoReverse(uint8_t speed, int8_t turn) {
    int8_t left = speed + turn;
    int8_t right = speed - turn;
    left = clamp(left, 0, 255);
    right = clamp(right, 0, 255);
    analogWrite(LPWM, left);
    analogWrite(RPWM, right);
    digitalWrite(LINA, LOW);
    digitalWrite(LINB, HIGH);
    digitalWrite(RINA, LOW);
    digitalWrite(RINB, HIGH);
}

inline int clamp(int value, int min, int max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
