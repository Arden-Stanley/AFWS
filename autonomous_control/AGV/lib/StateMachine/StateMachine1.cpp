#include "StateMachine1.h"
#include "LidarC1.h"

// Notes
// need to find a way to seperate this library from others more
// Need some type of yaw and pitch compensation
// Need pose struct that fuses gps with IMU with roll yaw pitch x,y,z with the refernce frames starting at (0,0,0)
// Expects a MotorControl class, Pose struct


float pi = 3.14159265359;

void StateMachine::Init() {
    mState = ManualDrive

}

// Main State-Machine Switch
void StateMachine::Run() {
    Switch (MainState) {
        case MainState::AutoDrive:
            if (Oldstate != AutoDrive)
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

void StateMachine::runAuto() {
    lidar.GetFullScan(2);
    pose.UpdatePose();
    StateMachine::CollisionDetection();
    switch (currentState) {
        case START:
            StateMachine::Start();
            break;
        case INBETWEEN_ROWS:
            StateMachine::InbetweenRows();
            break;
        case END_OF_ROW:
            StateMachine::EndofRow();
            break;
        case TURNING:
            StateMachine::Turning();
            break;
        case ALIGNING:
            StateMachine::Aligning();
            break;
        case COLLISION_AVOIDANCE:
            StateMachine::CollisionAvoidance();
            break;
    }
}
void StateMachine::CollisionDetection(){
    // Should i iterate through a for loop or add this to the lidarc1 library so it sets an interrupt
    if (m.distance < collisionThreshold) {
        mControl.Stop();
        currentState = COLLISION_AVOIDANCE;
    }
}

void StateMachine::Start() {
    // Apply brakes
    // Calibrate sensors
    // Zero out reference frame
    // Confirm starting state
    mControl.Stop();

}

void StateMachine::InbetweenRows() {
    // Confirm left and/or right rows
    // If no left or right rows switch to end of row
    // RANSAC
    // Create waypoint based on ransac
    // Drive to waypoint
    uint8_t i;
    rowPresent();
    if (!rightDetected && !leftDetected) {
        currentState = END_OF_ROW;
    }
    RansacConversion();
    // RANSAC
    // find middle line between two ransac lines with respect to field coordinate frame
    // create waypoint
    // ransacMidPoint is the offset from the center of AGV to midpoint between two ransac lines
    waypoint.x[waypoint.i] = pose.x + ransacMidPoint * sin(pose.yawRad);
    waypoint.y[waypoint.i] = pose.y + set.waypointOffset * cos(pose.yawRad);
    waypoint.i++;

}

void StateMachine::EndofRow() {
    // Mark Compass heading
    // Move out of row on current heading for z distance
    // switch to Turning State
    // How do decide which direction to turn?
    // find beginning of new row?
    //
    if (pose.compass > 180) {
        float EORmark = pose.compass - 180;
    }
    if (pose.compass < 180) {
        float EORmark = pose.compass + 180;
    }
    if ()
    DetectNextDirection();

}

void StateMachine::Turning() {
    // Make constant radius turn
    // Wait till 180 deg from previous compass marking
    // Change to aligning state
    if (pose.compass <= EORmark + 5 || pose.compass >= EORmark - 5) {
        currentState = ALIGNING;
    }
}

void StateMachine::Aligning() {
    if (leftDetected || rightDetected) {
        currentState = INBETWEEN_ROWS;
    }
    LookForward();
    // Maybe use old row coord + estimated row offset = new row coords
    // see if rows left and right then switch to inbetween rows

}

void StateMachine::CollisionAvoidance() {
    // Follow previous waypoints in reverse
    // Switch to previous state?
}

void RansacConversion () {
    for (uint16_t i; i < m.count; i++) {
        if (m.angle[i] > 0 && m.angle[i] < 180) {
            float rad = m.angle[i] * (pi/180);
            leftrow.x = m.distance[i] * sin(rad);
            leftrow.y = m.distance[i] * cos(rad);
        }
        if (m.angle[i] > 180 && m.angle[i] < 360) {
            float rad = m.angle[i] * (pi/180);
            leftrow.x = m.distance[i] * sin(rad);
            leftrow.y = m.distance[i] * cos(rad);
        }
    }
}

void RowPresent() {
    for (uint16_t i; i < m.count; i++) {
        // Is this done at compile or runtime?
        if (m.angle[i] > (90-set.detectionAngle/2) && m.angle < (90+set.detectionAngle/2) && m.distance[i] < set.detecctionThreshold) {
                uint8_t leftHit++;
        }
        if (m.angle[i] > (90-set.detectionAngle/2) && m.angle < (90+set.detectionAngle/2) && m.distnace[i] < set.detectionThreshold) {
                uint8_t rightHit++;
        }
    } // TODO: Change these so they are more readable using the other notation
    if (leftHit > set.hitThreshold) {
        bool leftDetected = true;
    } else {
        leftDetected = false;
    }
    if (rightHit > set.hitThreshold) {
        bool rightDetected = true;
    } else {
        rightDetected = false;
    }
}
