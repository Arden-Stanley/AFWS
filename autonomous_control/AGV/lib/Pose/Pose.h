



struct pose {
    float x;
    float y;
    float z;
    float pitch;
    float roll;
    float yaw;
};

class Position {
    public:
        bool safetyCheck;
        void PoseUpdate();
        pose p;
    private:
        void KalmanFilter();
};
