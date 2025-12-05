#include <Arduino.h>
#include "StateMachine.h"
#include "AGVPinDefs.h"

//Notes
// Ransac linefitting for row detection?
// Maybe do a heatmap for obstacle detection?
// IMU/GPS Fusion for better localization
// Kalman FIlter for IMU/GPS Fusion
// Deskewing Lidar Data




// Not sure if needed
void StateMachine::Start() {
    // (Do Once)
        // Reset Variables
        // GetLocalCoordPose
        // Get pointer to ScanData


}

// Main State-Machine Switch
void StateMachine::Run() {
    Switch (MainState) {
        case MainState::AutoDrive:
            StateMachine::RunAuto();
            break;

        case MainState::ManualDrive:
            StateMachine::RunManual();
            break;

        default:
            break;

    }
}

// Manual Control Code
void StateMachine::RunManual() {
    // Manual Motor Control
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
    // Manual Solenoid Control
    digitalWrite(RInnerSolenoid, agvrx.Output.RISolenoid ? HIGH : LOW);
    digitalWrite(LInnerSolenoid, agvrx.Output.LISolenoid ? HIGH : LOW);
    digitalWrite(ROuterSolenoid, agvrx.Output.ROSolenoid ? HIGH : LOW);
    digitalWrite(LOuterSolenoid, agvrx.Output.LOSolenoid ? HIGH : LOW);
    digitalWrite(ROuterSolenoid, agvrx.Output.ROSolenoid ? HIGH : LOW);
    digitalWrite(LOuterSolenoid, agvrx.Output.LOSolenoid ? HIGH : LOW);
    digitalWrite(OutPump, agvrx.Output.Pump ? HIGH : LOW);

    #ifdef Debug
    Serial.printf("LWForward: %d, LWReverse: %d, RWForward: %d, RWReverse: %d\n",
      agvrx.Output.LWForward,
      agvrx.Output.LWReverse,
      agvrx.Output.RWForward,
      agvrx.Output.RWReverse);
      delay(250);
    #endif


}

// AutoDrive Control Code
void StateMachine::RunAuto() {
    StateMachine::StateDetection();
    switch (CurrentState) {
        case INBETWEEN_ROWS:
            StateMachine::InbetweenRows();
            break;
        case END_OF_ROW:
            StateMachine::EndofRow();
            break;
        case COLLISION_AVOIDANCE:
            StateMachine::CollisionAvoidance();
            break;
        default:

            break;
    }
}

void StateMachine::InbetweenRows() {
    // Pointer to ScanData
    // IMU/GPS Fusion
    // Ransac Line Fit to find row edges
    // Create waypoints inbetween rows

}

void StateMachine::EndofRow() {
    // Pointer to ScanData
    // IMU/GPS Fusion
    // Find End of Row
    // Pass End of Row by z amount
    // Detect which side row is on
    // Turn towards next row using constant radius turn with row edge as reference
}

void StateMachine::CollisionAvoidance() {
    // Pointer to ScanData
    // Find clear path to reverse to previous waypoint?
    // Maybe follow previous path backwards

}

void StateMachine::StateDetection() {
    // Pointer to ScanData
    // IMU/GPS Fusion
    // Map Building
    // Sanity Checks
    // State Transition Logic
        //Inbetween Rows
            //
        //End of Row
            //
        //Collision Avoidance
            //
    if (StateMachine::InbetweenRowCondition) {
        CurrentState = CurrentState::INBETWEEN_ROWS;
    } else if (StateMachine::EndofRowCondition) {
        CurrentState = CurrentState::END_OF_ROW;
    } else if (StateMachine::CollisionAvoidanceCondition) {
        CurrentState = CurrentState::COLLISION_AVOIDANCE;
    }
}

bool StateMachine::InbetweenRowCondition() {
    // Logic to determine if inbetween rows
    return true;
}

bool StateMachine::EndofRowCondition() {
    // Logic to determine if at end of row
    return true;
}

bool StateMachine::CollisionAvoidanceCondition() {
    // Logic to determine if collision avoidance is needed
    return true;
}


void StateMachine::ResetALL() {
    // Reset all variables and states
}

void StateMachine::ResetManual() {
    // Reset variables and states after manual intervention
}