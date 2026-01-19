#define C1UART Serial1

#include "../include/RPLidarC1.h"
#include "../include/RANSAC.h"
#include "../include/StateMachine.h"
#include "../include/AGVPins.h"
#include <vector>
#include <deque>

using namespace std;

RPLidarC1 lidar(&Serial1);
RPLidarHealth health;

RPLidarMeasurement m;
RANSAC ransacRight;
RANSAC ransacLeft;

StateMachine S;

//Double Ended Queues for both sides of the car that the LiDAR is reading into
//Using Deques due its ability to remove form the from and back, which is needed for our rolling buffer 
deque<float> leftSideAngles, leftSideDistances;
deque<float> rightSideAngles, rightSideDistances;

//Thresh-hold for how far a wall should be before there needs to be gradual course correction 
float xsoftThreshhold = 127; //Reads in mm, 127mm = 5in

//Limits the amount in a deque to around 3-5 seconds worth of LiDAR data.
// 18000 (angle and distance) -> ~5 seconds. 
// 9000 per side, with 4500 split between angle in distance.
static size_t dequeLimit = 4500;

//bool obstacleDetected = false;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  delay(2000);

  pins p;

  pinMode(p.PWM1, OUTPUT);
  pinMode(p.PWM2, OUTPUT);
  pinMode(p.INA1, OUTPUT);
  pinMode(p.INA2, OUTPUT);
  pinMode(p.INB1, OUTPUT);
  pinMode(p.INB2, OUTPUT);

  Serial.println("Initializing RPLidar...");
  if (!lidar.begin(460800, 4000)) {
    Serial.println("Failed to initialize RPLidar.");
    while (1) delay(1000);
  }

  if(lidar.get_health(&health)) {
    lidar.print_health(&health);
    if(health.status != RPLIDAR_STATUS_OK) {
      Serial.println("Warning: LiDAR not healthy, attempting reset...");
      lidar.reset();
      delay(2000);
    }
  }
  else
    Serial.println("Failed to get health status.");

  if(!lidar.start_scan()){
    Serial.println("Failed to start scan");
    while(1) delay(1000);
  }
  Serial.println("Scan started successfully");

  S.STATE = S.STOP;
}

void loop() {
  // Vectors to hold the converted LiDAR points
  vector<points> rightCartesianConverted;
  vector<points> leftCartesianConverted;

  if (!lidar.get_measurement(&m)) return;

  //Checks if detect angles are within a certain thresh-hold.
  //45 -> 135 degrees is right side
  //225 -> 315 is left side.
  if (m.angle > 45 && m.angle <= 135) {
    rightSideAngles.push_back(m.angle);
    rightSideDistances.push_back(m.distance);
  } 
  else if (m.angle > 225 && m.angle <= 315) {
    leftSideAngles.push_back(m.angle);
    leftSideDistances.push_back(m.distance);
  }

  //If the size of one of the angle deques is greater than the dequeLimit (4500)...
  //Start a rolling buffer that will delete the beginning of the deque and add another data point.
  if (rightSideAngles.size() >= dequeLimit || leftSideAngles.size() >= dequeLimit) {
    rightSideAngles.pop_front();
    rightSideDistances.pop_front();
    leftSideAngles.pop_front();
    leftSideDistances.pop_front();
  }

  // This will only start the RANSAC and STATEMACHINE process whennever a rotation starts 
  if (m.start_flag) {
    // Wait until there is at-least 50 data points for both sides
    if (leftSideAngles.size() > 50 || rightSideAngles.size() > 50) {
    // Clear vectors
      leftCartesianConverted.clear();
      rightCartesianConverted.clear();

    // Convert r and thada to x and y for RANSAC
      ransacRight.cartesianConversion(rightSideAngles, rightSideDistances, rightCartesianConverted);
      ransacLeft.cartesianConversion(leftSideAngles, leftSideDistances, leftCartesianConverted);

    // Try to find best-fit line through RANSACLoop
      ransacLeft.RANSACLoop(leftCartesianConverted);
      ransacRight.RANSACLoop(rightCartesianConverted);

    // Calculate distance from origin (the car) to the RANSAC lines
      ransacRight.distancetoLine();
      float rightDistance = ransacRight.distance;

      ransacLeft.distancetoLine();
      float leftDistance = ransacLeft.distance;

      // Calculation for finding the distance between RANSAC lines. Use this for PID as an error to correct.
      float lineDifference = rightDistance - leftDistance;

      // Determines length for how long the car remains stopped.
      static unsigned long stopStartTime = 0;

      switch(S.STATE) {
      //Stop, wait for 5 seconds, check line validation. If valid, switch to inbetween_rows.
        case S.STOP:
          S.stop(); 
          if (stopStartTime == 0) 
            stopStartTime = millis(); 

          if (millis() - stopStartTime > 5000) {
            if (ransacRight.lineValidation() || ransacLeft.lineValidation()) {
              S.STATE = S.INBETWEEN_ROWS;
              stopStartTime = 0; 
              Serial.println("WALL FOUND: SWITCHING TO INBETWEEN_ROWS");
            }
          }
        break;

        case S.INBETWEEN_ROWS:
        // Use the difference between walls to steer via PID (inside the state machine)
          S.inbetween_rows(lineDifference);
          break;
      }
    }
  }
}







