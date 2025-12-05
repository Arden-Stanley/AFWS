#pragma once

struct AGVPinDefs {
    int LINA, LINB, LPWM;
    int RINA, RINB, RPWM;
    int LInnerSol;
    int RInnerSol;
    int LOuterSol;
    int ROuterSol;
    int Pump;
    int LidarTX, LidarRX;
    int GPSRX, GPSTX;
    int IMUTX, IMURX;
};