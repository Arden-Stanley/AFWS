#include <Arduino.h>
#include "AGRIS.h"
#include "LidarC1.h"
// Notes
// Add timeout if autodrivepacket not recieved in time
// Added DMAMEM for the giant arrays of scan data

// Create Agris Object
AGRIS agvrx;
// Create Lidar Objet
LidarC1 lidar(Serial1);
// Define Setup for AGRIS
#define BaudRate 115200
#define TX_PIN 14
#define RX_PIN 15
// Define Lidar memory
#define MAXPOINTS 1000
#define MAXCYCLES 5
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
bool LidarHealth = false;
bool LidarScan = false;

void setup() {
Serial.begin(115200);
agvrx.begin(BaudRate, TX_PIN, RX_PIN);
lidar.begin(460800);
lidar.GetHealth();
if (lidar.Health.status == 0) {
  lidar.StartScan();
}
pinMode(13, OUTPUT); // Onboard LED
pinMode(LINA, OUTPUT);
pinMode(LINB, OUTPUT);
pinMode(LPWM, OUTPUT);
pinMode(RINA, OUTPUT);
pinMode(RINB, OUTPUT);
pinMode(RPWM, OUTPUT);

}


void loop() {
  agvrx.RX();
  if (agvrx.AutoDriveState.AutoDrive == true) {
    if (lidar.Health.status == 0) {
      lidar.GetSingleScan();
      Serial.printf("Quality: %d, X: %d, Y: %d\n", lidar.Measurements[0].quality[0], lidar.Measurements[0].NewXCoord[0], lidar.Measurements[0].NewYCoord[0]);
    }
    


    

  }

  else if (agvrx.AutoDriveState.AutoDrive == false) {
    // You can ignore most of anything down here this is just manual drive code
    digitalWrite(13, LOW);
    
    #ifdef SolenoidTest
    digitalWrite(RInnerSolenoid, agvrx.Output.RISolenoid ? HIGH : LOW);
    digitalWrite(LInnerSolenoid, agvrx.Output.LISolenoid ? HIGH : LOW);
    digitalWrite(ROuterSolenoid, agvrx.Output.ROSolenoid ? HIGH : LOW);
    digitalWrite(LOuterSolenoid, agvrx.Output.LOSolenoid ? HIGH : LOW);
    digitalWrite(ROuterSolenoid, agvrx.Output.ROSolenoid ? HIGH : LOW);
    digitalWrite(LOuterSolenoid, agvrx.Output.LOSolenoid ? HIGH : LOW);
    digitalWrite(OutPump, agvrx.Output.Pump ? HIGH : LOW);
    #endif

    // Manual Drive Motor Control
    if (agvrx.Output.LWForward > 0 && agvrx.Output.LWReverse == 0) {
      analogWrite(LPWM,agvrx.Output.LWForward);
      digitalWrite(LINA, HIGH);
      digitalWrite(LINB, LOW);
    } else if (agvrx.Output.LWReverse > 0 && agvrx.Output.LWForward == 0) {
      analogWrite(LPWM,agvrx.Output.LWReverse);
      digitalWrite(LINB, HIGH);
      digitalWrite(LINA, LOW);
    } else {
      digitalWrite(LINA, LOW);
      digitalWrite(LINB, LOW);
      digitalWrite(LPWM, LOW);
    }
    if (agvrx.Output.RWForward > 0 && agvrx.Output.RWReverse == 0) {
      analogWrite(RPWM, agvrx.Output.RWForward);
      digitalWrite(RINA, HIGH);
      digitalWrite(RINB, LOW);
    } else if (agvrx.Output.RWReverse > 0 && agvrx.Output.RWForward == 0) {
      analogWrite(RPWM,agvrx.Output.RWReverse);
      digitalWrite(RINB, HIGH);
      digitalWrite(RINA, LOW);
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
    #endif
  }
}



