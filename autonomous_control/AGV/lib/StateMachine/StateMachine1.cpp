#include "StateMachine1.h"


void StateMachine::start() {

}

void StateMachine::runAuto() {
    switch (currentState) {
        case START:
            StateMachine::startState();
            break;
        case INBETWEEN_ROWS:
            StateMachine::inbetweenRows();
            break;
        case END_OF_ROW:
            StateMachine::endofRow();
            break;
        case TURNING:
            StateMachine::turning();
            break;
        case ALIGNING:
            StateMachine::aligning();
            break;
        default:

    }
}

void StateMachine::startState() {
    lidar.GetFullScan(1);
    Pose.getPose();
    // Maybe detect starting state
}

void StateMachine::inbetweenRows() {
    lidar.GetFullScan(1);
    Pose.updatePose;
    StateMachine::rowPresent();
    if (!leftRow && !rightRow) {
        currentState = END_OF_ROW;
        eorMark =
    }
    // Build RANSAC lines
    // Create waypoint
    // Save waypoint
}

void StateMachine::endofRow() {
    lidar.GetFullScan(1)
    Pose.updatePose;
    // Go out z distance
    //
    // set turning direction
    if (distanceTravelled)
    currentState = TURNING;
}

void StateMachine::turning() {
    // Locate new row
    // begin to turn at constant radius turn around corn
    //
}

void StateMachine::aligning() {
    lidar.GetFullScan(1);
    Pose.updatePose;
    // Look forward for new row and align with it
    //
}
