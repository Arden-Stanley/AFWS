#pragma once
#include <Arduino.h>

#ifndef MAXPOINTS
#define MAXPOINTS 1000
#endif

#ifndef MAXCYCLES
#define MAXCYCLES 5
#endif

struct ParserData{
    uint32_t timeStamp;
    bool startFlag;
    bool invFlag;
    uint8_t quality;
    float angle;
    float distance;
    float newX;
    float newY;
};

struct MeasurementData{
    uint32_t timeStamp[MAXPOINTS];
    uint16_t count;
    uint32_t time;
    uint8_t quality[MAXPOINTS];
    float angle[MAXPOINTS];
    float distance[MAXPOINTS];
    float NewXCoord[MAXPOINTS];
    float NewYCoord[MAXPOINTS];
};

struct HealthData {
    uint8_t status;
    uint16_t errorCode;
};

class LidarC1 {
    public:
        LidarC1(HardwareSerial& serial);
        void begin(uint32_t baudrate);
        void GetHealth();
        void StartScan();
        bool GetDescriptor(uint8_t expectedDataType);
        bool GetHealthResponse();
        bool ReadScanPacket();
        void ParseScanPacket(const uint8_t Packet[5]);
        bool GetSingleScan();
        bool GetFullScan(uint8_t NumofCycles);
        void serialClear();
        MeasurementData Measurements[MAXCYCLES];
        HealthData Health;
    private:
        ParserData Parser;
        HardwareSerial& serial;
        uint32_t serialTimeout = 1000;
        uint32_t TCMD = 0;
        bool Scanning = false;
};



