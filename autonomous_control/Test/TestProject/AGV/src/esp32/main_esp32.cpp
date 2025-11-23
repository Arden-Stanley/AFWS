#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include "AGRIS.h"


// Create Xbox Controller Object
XboxSeriesXControllerESP32_asukiaaa::Core xboxController; 

// Create AGRIS Object
AGRIS agvtx;

// Define Serial setup
#define RX_PIN 2
#define TX_PIN 1
#define TX_BAUD 115200


void setup() {
  Serial.begin(115200);
  xboxController.begin();
  agvtx.begin(TX_BAUD,TX_PIN, RX_PIN);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  xboxController.onLoop(); // Need to look more at this from library
  if (xboxController.isConnected()) { 
    // Turn on LED when connected
    digitalWrite(LED_BUILTIN, HIGH);
    // Get the data from the controller
    agvtx.GetControllerData();
    // Set AutoDriveState
    agvtx.SetAutoDrive();
    // Send Controller Data
    agvtx.TX();
    #ifdef Debug
      Serial.printf("LSForward: %d, LSReverse: %d, RSForward: %d, RSReverse: %d, LStick: %d, RStick: %d \n",
      agvtx.Input.LSForward,
      agvtx.Input.LSReverse,
      agvtx.Input.RSForward,
      agvtx.Input.RSReverse,
      xboxController.xboxNotif.joyLVert,
      xboxController.xboxNotif.joyRVert);
      delay(250);
    #endif
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
    #ifdef Debug
    Serial.println("not connected");
    delay(500);
    #endif
  }
}
