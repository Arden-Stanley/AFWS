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

StateMachine S;

//Double Ended Queues for both sides of the car that the LiDAR is reading into
//Using Deques due its ability to remove form the from and back, which is needed for our rolling buffer 
deque<float> leftSideAngles, leftSideDistances;
deque<float> rightSideAngles, rightSideDistances;

//Thresh-hold for how far a wall should be before there needs to be gradual course correction 
float xsoftThreshhold = 127; //Reads in mm, 127mm = 5in

//Limits the amount in a vector to around 1-2 seconds worth of LiDAR data.
// 18000 (angle and distance) -> ~5 seconds. 
// 9000 per side, with 4500 split between angle in distance.
static size_t dequeLimit = 4500;

bool obstacleDetected = false;

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
  vector<points> rightCartesianConverted;
  vector<points> leftCartesianConverted;

  RPLidarMeasurement m;
  RANSAC ransacRight;
  RANSAC ransacLeft;
  if (!lidar.get_measurement(&m)) return;

  //Detects the start of the rotation
  if(m.start_flag){
    while(lidar.is_scanning()){
    //Checks if detect angles are within a certain thresh-hold.
    //0 -> 180 degrees is right side
    //181 -> 359 is left side.
      if(m.angle > 0 && m.angle <= 180){
        rightSideAngles.push_back(m.angle);
        rightSideDistances.push_back(m.distance);
        ransacRight.cartesianConversion(rightSideAngles, rightSideDistances, rightCartesianConverted);
      }
      else if(m.angle > 180 && m.angle <= 359){
        leftSideAngles.push_back(m.angle);
        leftSideDistances.push_back(m.distance);
        ransacLeft.cartesianConversion(leftSideAngles, leftSideDistances, leftCartesianConverted);
      }

    //If the size of one of the angle deques is greater than the dequeLimit (4500)...
    //Start a rolling buffer that will delete the beginning of the deque and add another data point.
      if(rightSideAngles.size() >= dequeLimit || leftSideAngles.size() >= dequeLimit){
        rightSideAngles.pop_front();
        rightSideDistances.pop_front();
        leftSideAngles.pop_front();
        leftSideDistances.pop_front();
      }

    //Puts the converted LiDAR points into the RANSAC process
      ransacLeft.RANSACLoop(leftCartesianConverted);
      ransacRight.RANSACLoop(rightCartesianConverted);

    //Calculates the distance from the origin (the LiDAR) to the RANSAC calculated line
      ransacRight.distancetoLine();
      float rightDistance = ransacRight.distance;

      ransacLeft.distancetoLine();
      float leftDistance = ransacLeft.distance;

      float lineDifference = fabs(rightDistance - leftDistance);

    //Calculates the heading angle and error in the distance from the soft threshhold to the previously calculated distance
    //Need to incorporate this with PID for added autonomy
      ransacRight.headingAngle(xsoftThreshhold);
      ransacLeft.headingAngle(xsoftThreshhold);

    //Checks if the RANSAC found line is valid
      bool rightValidation = ransacRight.lineValidation();
      bool leftValidation = ransacLeft.lineValidation();

      StateMachine S;
      S.STATE = S.STOP;

    // Decided that PID (Proportional Integral Derivative) control is the best option.
      switch(S.STATE){
        //Stop, wait for 5 seconds, check line validation. If valid, switch to inbetween_rows.
        case S.STOP:
          S.stop();
          delay(5000);
          if(rightValidation == 1 || leftValidation == 1){
            S.STATE = S.INBETWEEN_ROWS;
          }
          break;

        case S.INBETWEEN_ROWS:
          S.inbetween_rows(lineDifference);

      }
    }  
  }    
}