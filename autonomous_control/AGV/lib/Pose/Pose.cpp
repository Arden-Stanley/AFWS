#include "Pose.h"


// Notes
// Need to add GPS
// Need to add IMU
// Need to add kalman filter
//


Position pose;

void Pose::Init() {

}

void Pose::UpdatePose() {

}

float Pose::DistanceTravelled() {
    float xDistance = (pose.newX - pose.oldX)^2;
    float yDistance = (pose.newY - pose.oldY)^2;
    return sqrt(xDistance+yDistance);
}
