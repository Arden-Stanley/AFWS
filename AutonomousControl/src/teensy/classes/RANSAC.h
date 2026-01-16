#ifndef RANSAC_H
#define RANSAC_H

#include <vector>
#include <math.h>

/*RANSAC DESCRIPTION
STEPS:
1: Convert LiDAR angle and radius to x and y (cartesian conversion)
2: Take two random converted data points. (RANSACloop)
3: Put them into a line equation, normalize it, and see if other points fit.(fitter, pointLine)
4: Finish the loop and find the best line (RANSACloop)
4: Calculate the distance from the car to the calculated inlier line. (distancetoLine)
5: Calculate heading angle and error (headingAngle)
6: Validate the line for the state-machine (lineValidation)
*/

//Going to change this soon
const int MAX_ITERATIONS = 200;
//Everything within a 3 inch thresh-hold of the line will be considered as an inlier
const float DIST_THRESHOLD = 75; // 75mm = 2.95 inches. 
const int MIN_INLIERS = 20; 

struct points {
  float x;
  float y;
};

struct plane {
  float a;
  float b;
  float c;
};

class RANSAC {

  public: 
    void cartesianConversion(const std::vector<float>& angles, const std::vector<float>& ranges, std::vector<points>& points);
    plane fitter(const points& p1, const points& p2);
    float pointLine(const plane& line, const points& p);
    void RANSACLoop(const std::vector<points>& points);
    void distancetoLine();
    void headingAngle(float targetDistance);
    bool lineValidation();

    int bestInlierCount;
    float distance;
    float headingError;
    float distanceError;
    plane bestLine;
  
  private:
    plane currentLine;
};

#endif