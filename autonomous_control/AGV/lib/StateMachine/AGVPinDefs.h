#pragma once

struct AGVPinDefs {
    int L_INA, L_INB, L_PWM;
    int R_INA, R_INB, R_PWM;
    int LInnerSol, LOuterSol;
    int RInnerSol, ROuterSol;
    int Pump;
    int Lidar_RX, Lidar_TX;
    int GPS_RX, GPS_TX;
    int Comp_SDA, Comp_SCL;
    int IMU_SDA, IMU_SCL;
};