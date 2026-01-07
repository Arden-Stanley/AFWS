#include <Arduino.h>
#include "StateMachine.h"
#include "AGVPinDefs.h"
#include "LidarC1.h"

//Notes
// Ransac linefitting for row detection?
// Maybe do a heatmap for obstacle detection?
// IMU/GPS Fusion for better localization
// Kalman Filter for IMU/GPS Fusion
// Deskewing Lidar Data
// Do we want to constantly check state?

#define agvBubble 254 // Bubble around AGV in mm


LidarC1 lidar(Serial1)


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
            currentState = START;
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
            // Defaults when no row detected on either side
            // Might cause an issue with the end of row state moving so far out

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
    // Detect which side row is on (use leftRow and rightRow?)
    // Turn towards next row using constant radius turn with row edge as reference

}

void StateMachine::Turning() {

}

void StateMachine:: Alignment() {

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
        //Start
            //
        //Inbetween Rows
            //
        //End of Row
            //


    // TODO: pose Library?
    Lidar.GetFullScan(1);
    RowPresent();
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
        if (leftRow || rightRow) { // Both Left and Right Detected
            return true;
        }
        if (LeftRow) {
            return true;
        }
        return false;
}

bool StateMachine::EndofRowCondition() {
    // Logic to determine if at end of row
    if (!leftRow || !rightRow) {
        return true;
    }
    return false;
}

bool StateMachine::CollisionAvoidanceCondition() {
    // Logic to determine if collision avoidance is needed
    // Might be better to do collision detection in lidar library so we dont need to do a for loop and we can check it as we get the data
    // Using polar lidar data
    #ifdef DISTANCE
    uint16_t* mPtr = Measurement.Distance
    for (uint16_t i = 0; i < sizeof(mPtr); i++) {
        if (*mptr[i] < agvBubble) {
            return true;
        }
        return false;
    }
    #endif

    // TODO: Using rectangular coord data
    #ifdef RECTANGULAR
    collisionDistance = 0;
    if () {
    return true;
    };
    return false;
    #endif

void StateMachine::ResetALL() {
    // Reset all variables and states
}

void StateMachine::ResetManual() {
    // Reset variables and states after manual intervention
}

// Might need to break this into functions one for left and one for right
bool RowPresent() {
    // leftMin = minimum angle for left side of scanner
    // leftMax = maximum angle for left side of scanner
    // rightMin = minimum angle for right side of scanner
    // rightMax = maximum angle for right side of scanner
    // threshold = theshold distance for it to be counted as a hit
    // minHits = minimum number of hits for function to be satisfied that there is a row to the left or right
    uint8_t leftHit = 0;
    uint8_t rightHit = 0;
    for (uint16_t i = 0; i < sizeof(Measurements[0].Angle); i++) {
        if (Measurements[0].Angle[i] > leftMin && Measurements[0].Angle[i] < leftMax) {
            if (Measurement[0].Distance[i] < threshold) {
                leftHit++;
            }
        }
        if (Measurements[0].Angle[i] > rightMin && Measurements[0].Angle[i] < rightMax)
            if (Measurements[0].Distance[i] < threshold) {
                rightHit++;
            }
    }
    if (leftHit > minHits) {
        bool leftRow = true;
    } else {
        leftRow = false;
    }
    if (rightHit > minHits) {
        bool rightRow = true;
    } else {
        leftRow = false;
    }
}
