#define C1UART Serial1

#include "RPLidarC1.h"
#include "RANSAC.h"
#include "StateMachine.h"
#include "AGVPins.h"
#include <vector>

using namespace std;

RPLidarC1 lidar(&Serial1);
RPLidarHealth health;

//Vectors for both sides of the car that the LiDAR is reading into
vector<float> leftSideAngles;
vector<float> leftSideDistances;
vector<float> rightSideAngles;
vector<float> rightSideDistances;

//Origin for the RANASC 2-D Plane where the car is centered at (0,0)
const float carX = 0;
const float carY = 0;

//Thresh-hold for how far a wall should be before there needs to be gradual course correction 
float xsoftThreshhold = 127; //Reads in mm, 127mm = 5in
//Thresh-hold for how far a wall should be before there needs to be immediate course correction
float xhardThreshold = 50.8; //Reads in mm, 50.8mm = 2in

//Limits the amount in a vector to around 1-2 seconds worth of LiDAR data.
// 18000 (angle and distance) -> ~5 seconds. 
// 9000 per side, with 4500 split between angle in distance.
static size_t vectorLimit = 4500;

bool obstacleDetected = false;

void setup() {
  // put your setup code here, to run once:
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
}

void loop() {
  // put your main code here, to run repeatedly:
  vector<points> rightCartesianConverted;
  vector<points> leftCartesianConverted;

  RPLidarMeasurement m;
  RANSAC ransac;
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
        ransac.cartesianConversion(rightSideAngles, rightSideDistances, rightCartesianConverted);
      }
      else if(m.angle > 180 && m.angle <= 359){
        leftSideAngles.push_back(m.angle);
        leftSideDistances.push_back(m.distance);
        ransac.cartesianConversion(leftSideAngles, leftSideDistances, leftCartesianConverted);
      }

    //If the size of one of the angle vectors is greater than the vectorLimit (4500)...
    //Start a rolling buffer that will delete the beginning of the vector and add another data point.
      if(rightSideAngles.size() >= vectorLimit || leftSideAngles.size() >= vectorLimit){
        rightSideAngles.erase(rightSideAngles.begin());
        rightSideDistances.erase(rightSideDistances.begin());
        leftSideAngles.erase(leftSideAngles.begin());
        leftSideDistances.erase(leftSideDistances.begin());
      }

    //Puts the converted LiDAR points into the RANSAC process
      ransac.RANSACLoop(leftCartesianConverted);
      ransac.RANSACLoop(rightCartesianConverted);

    //Calculates the distance from the origin (the LiDAR) to the RANSAC calculated line
      ransac.distancetoLine();
      float distance = ransac.distance;

    //Calculates the heading angle and error in the distance from the soft threshhold to the previously calculated distance
      ransac.headingAngle(xsoftThreshhold);

    //Checks if the RANSAC found line is valid
      bool validation = ransac.lineValidation();

      StateMachine S;
      
      S.STATE = S.STOP;

      switch(S.STATE){
        //Stop, wait for 5 seconds, check line validation. If valid, switch to inbetween_rows.
        case S.STOP:
          S.stop();
          delay(5000);
          if(validation == 1){
            S.STATE = INBETWEEN_ROWS;
          }
          break;

        case S.INBETWEEN_ROWS:

      }
    }  
  }    
}