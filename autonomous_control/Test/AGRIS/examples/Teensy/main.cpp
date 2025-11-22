#include <Arduino.h>
#include "AGRIS.h"
// Notes
// Add timeout if autodrivepacket not recieved in time


// Create Agris Object
AGRIS agvrx;

// Define Setup for AGRIS
#define BaudRate 115200
#define TX_PIN 14
#define RX_PIN 15
// Motor Driver Pin Definitions
#define LINA 7
#define LINB 8
#define LPWM 9
#define RINA 4
#define RINB 5
#define RPWM 6
// Sprayer Pin definitions
#define RInnerSolenoid 
#define LInnerSolenoid 
#define ROuterSolenoid 
#define LOuterSolenoid 
#define OutPump 

// Initialize Variables
bool AutoDrive = false;


void setup() {
Serial.begin(115200);
agvrx.begin(BaudRate, TX_PIN, RX_PIN);
pinMode(13, OUTPUT); // On-board LED

}




void loop() {
  agvrx.RX();
  if (agvrx.AutoDriveState.AutoDrive == true) {
    // AutoDrive Code
    digitalWrite(13, HIGH);
    Serial.printf("AutoDrive: %d\n", agvrx.AutoDriveState.AutoDrive);
    delay(250);









  }
  else if (agvrx.AutoDriveState.AutoDrive == false) {
    // You can ignore most of anything down here this is just manual drive code

    digitalWrite(13, LOW);
    // ManualDriveCode
    // digitalWrite(RInnerSolenoid, agvrx.Output.RISolenoid ? HIGH : LOW);
    // digitalWrite(LInnerSolenoid, agvrx.Output.LISolenoid ? HIGH : LOW);
    // digitalWrite(ROuterSolenoid, agvrx.Output.ROSolenoid ? HIGH : LOW);
    // digitalWrite(LOuterSolenoid, agvrx.Output.LOSolenoid ? HIGH : LOW);
    // Testing
    //digitalWrite(ROuterSolenoid, agvrx.Output.ROSolenoid ? HIGH : LOW);
    //digitalWrite(LOuterSolenoid, agvrx.Output.LOSolenoid ? HIGH : LOW);
    //digitalWrite(OutPump, agvrx.Output.Pump ? HIGH : LOW);





    // Manual Drive Motor Control
    if (agvrx.Output.LWForward > 0 && agvrx.Output.LWReverse == 0) {
      analogWrite(LPWM,agvrx.Output.LWForward);
      digitalWrite(LINA, HIGH);
    } else if (agvrx.Output.LWReverse > 0 && agvrx.Output.LWForward == 0) {
      analogWrite(LPWM,agvrx.Output.LWReverse);
      digitalWrite(LINB, HIGH);
    } else {
      digitalWrite(LINA, LOW);
      digitalWrite(LINB, LOW);
      digitalWrite(LPWM, LOW);
    }
    if (agvrx.Output.RWForward > 0 && agvrx.Output.RWReverse == 0) {
      analogWrite(RPWM, agvrx.Output.RWForward);
      digitalWrite(RINA, HIGH);
    } else if (agvrx.Output.RWReverse > 0 && agvrx.Output.RWForward == 0) {
      analogWrite(RPWM,agvrx.Output.RWReverse);
      digitalWrite(RINB, HIGH);
    } else {
      digitalWrite(RINA, LOW);
      digitalWrite(RINB, LOW);
      digitalWrite(RPWM, LOW);
    }


    #ifdef Debug
    Serial.printf("LWForward: %d, LWReverse: %d, RWForward: %d, RWReverse: %d\n",
      agvrx.Output.LWForward,
      agvrx.Output.LWReverse,
      agvrx.Output.RWForward,
      agvrx.Output.RWReverse);
      delay(250);




  Serial.printf("LStick:%d  RStick:%d  LB:%d  RB:%d  LT:%d  RT:%d AutoDriveState:%d\n",
    agvrx.Output.RWheel,
    agvrx.Output.LWheel,
    agvrx.Output.RISolenoid,
    agvrx.Output.LISolenoid,
    agvrx.Output.ROSolenoid,
    agvrx.Output.LOSolenoid,
    agvrx.AutoDriveState.AutoDrive);
    #endif
  }




}

