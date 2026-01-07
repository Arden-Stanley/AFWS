#include <Arduino.h>

struct line {
    float m;
    float b;
    int inliers;
    bool isValid;
}

struct points {
        float x;
        float y;
    }

class RANSAC {
    public:
        void GetBestLine();
        void CartesianConversion();
    private:
        void FitLine();
        float PointtoLine();

}
