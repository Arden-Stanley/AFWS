#include <cstdint>


class MotorControl {
   public:
        void Forward(uint8_t Speed);
        void Reverse(uint8_t Speed);
        void Stop();
        void LeftTurn(uint8_t Intensity);
        void RightTurn(uint8_t Intensity);
        void AutoForward(uint8_t speed, int8_t turn);
        void AutoReverse(uint8_t speed, int8_t turn)

};
