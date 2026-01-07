

struct line {
    float m;
    float b;
};

struct waypoint {
    float x;
    float y;
};

class Perception {
    public:
        Perception();
        void UpdatePerception();
        bool CheckCollision();
        void GetWaypoint();
        waypoint wp;
    private:
        void RANSAC();

};
