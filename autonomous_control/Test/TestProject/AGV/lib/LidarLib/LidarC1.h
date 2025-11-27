#pragma once
#include <Arduino.h>

struct LidarDescriptor {
    uint8_t DescriptorPacket[7];
    uint32_t DataLength;
    uint8_t SendMode;
    uint8_t DataType;
};

struct LidarScanData {
    uint32_t Distance[720];
    uint32_t Angle[720];
    uint8_t Quality[720];
};

struct LidarHealthData {
    uint8_t Status;
    uint16_t ErrorCode;
};





class LidarC1 {
    public:
        void begin(int BaudRate);
        void GetHealth();
        void Stop();
        void Reset();
        void StartScan();
        void MotorSpeed(uint16_t RPM);
        void ReadScan();
        LidarDescriptor Descriptor;
        LidarScanData ScanData;
        LidarHealthData HealthData;
        uint8_t CycleIDX = 0;
    private:
        void ParseDescriptor(uint8_t* Buffer);
        void ParseScanData(uint8_t* Buffer, uint8_t DataLength);
        void ParseHealthData(uint8_t* Buffer, uint8_t DataLength);
        uint32_t TCMD = 0;
        uint8_t ScanState = 0;
        uint8_t ReadByte;
        uint8_t Buffer[256];
        uint32_t DataLength = 0;
        uint8_t DataType = 0;
        uint8_t i = 0;
        uint8_t IDX = 0;
        uint8_t MIDX = 0;
        uint8_t HealthPacket[2];
        uint8_t StopPacket[2];
        uint8_t ResetPacket[2];
        uint8_t ScanTXPacket[2];
        uint8_t SpeedPacket[5];
};