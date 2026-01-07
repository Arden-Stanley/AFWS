#include <Arduino.h>
#include "LidarC1.h"

#define Debug

// Notes
// Have to define max point count and max cycle count above #include for memory allocation
// Might switch to circular buffer for continuous data retieval
// IMU/GPS Deskew

// Start and Check Bytes
#define StartByte 0xA5
#define CheckByte 0x5A

// Commands
#define STOPCMD 0x25
#define RESETCMD 0x40
#define SCANCMD 0x20
#define GETHEALTHCMD 0x52
#define MOTORSPEEDCMD 0xA8

// Data Types
#define SCANDATATYPE 0x81
#define HEALTHDATATYPE 0x04

// Delays
#define HEALTHDELAY 20
#define RESETDELAY 2000
#define SCANDELAY 20
#define STOPDELAY 20
#define MOTORDELAY 20

const float pi = 3.14159265;

DMAMEM MeasurementData LidarC1::Measurements[MAXCYCLES];

LidarC1::LidarC1(HardwareSerial& serial) : serial(serial) {

}

void LidarC1::begin(uint32_t baudrate) {
    serial.begin(baudrate);
    serialTimeout = 1000;
    serialClear();
    Scanning = false;
}

void LidarC1::GetHealth() {
    if (millis() > TCMD) {
        const uint8_t getHealthCmd[2] = {StartByte, GETHEALTHCMD};
        serial.write(getHealthCmd, 2);
        TCMD = millis() + HEALTHDELAY;

        if(!GetDescriptor(HEALTHDATATYPE)) {
            Serial.println("Failed to get health descriptor");
            return;
        }
        if(!GetHealthResponse()) {
            Serial.println("Failed to get health response");
            return;
        }
        #ifdef Debug
            Serial.print("Lidar Health Status: ");
            Serial.println(Health.status);
            Serial.print(Health.errorCode);
        #endif
    } 
    else {
        Serial.println("Waiting for command delay");
    }
}

void LidarC1::StartScan() {
    if (millis() > TCMD) {
        const uint8_t scanCmd[2] = {StartByte, SCANCMD};
        serial.write(scanCmd, 2);
        TCMD = millis() + SCANDELAY;

        if (!GetDescriptor(SCANDATATYPE)) {
            Serial.println("Failed to get scan descriptor");
            return;
        }
        Scanning = true;
    } else {
        Serial.println("Waiting for command delay");
    }
}

void LidarC1::Stop() {
    if (millis() > TCMD) {
        const uint8_t stopCmd[2] = {StartByte, STOPCMD};
        serial.write(stopCmd, 2);
        TCMD = millis() + STOPDELAY;
        Scanning = false;
    }
}

void LidarC1::Reset() {
    if (millis() > TCMD) {
        const uint8_t resetCmd[2] = {StartByte, RESETCMD};
        serial.write(resetCmd, 2);
        TCMD = millis() + RESETDELAY;
        Scanning = false;
    }
}

// Get_SampleRate


void LidarC1::MotorSpeed(uint16_t RPM) {
    if (millis() > TCMD) {
        uint8_t speedCmd[5];
        speedCmd[0] = StartByte;
        speedCmd[1] = MOTORSPEEDCMD;
        speedCmd[2] = RPM >> 8 & 0xFF;
        speedCmd[3] = RPM & 0xFF;
        speedCmd[4] = 1;
        serial.write(speedCmd, 5);
        TCMD = millis() + MOTORDELAY;
    }
}

bool LidarC1::GetDescriptor(uint8_t expectedDataType) {
    uint8_t dWindow[7];
    uint8_t index = 0;
    uint32_t Start = millis();
    while (millis() - Start < serialTimeout) {
        if (serial.available()) {
            dWindow[index++] = serial.read();
            if (index == 7) {
                if (dWindow[0] == StartByte && dWindow[1] == CheckByte && dWindow[6] == expectedDataType) {
                    return true;
                }
                memmove(dWindow, dWindow + 1, 6);
                index = 6;
            }

        }
    }
    return false;
}

bool LidarC1::GetHealthResponse() {
    uint8_t mLength = 3;
    uint8_t buffer[3];
    uint8_t index = 0;
    uint32_t Start = millis();
    while (millis() - Start < serialTimeout) {
        if (serial.available()) {
            buffer[index++] = serial.read();
            if (index == mLength) {
                Health.status = buffer[0];
                Health.errorCode = (buffer[2] << 8) | buffer[1];
                return true;
            }
        }
    }
    return false;
}

bool LidarC1::ReadScanPacket() {
    uint8_t sWindow[5];
    uint8_t Index = 0;
    uint32_t Start = millis();
    while (millis() - Start < serialTimeout) {
        if (serial.available()) {
            sWindow[Index++] = serial.read();
            if (Index == 5) {
                Parser.startFlag = (sWindow[0] & 0x01) != 0;
                Parser.invFlag = (sWindow[0] & 0x02) != 0;
                if (Parser.startFlag != Parser.invFlag) {
                    ParseScanPacket(sWindow);
                    return true;
                }  
                memmove(sWindow, sWindow + 1, 4);
                Index = 4;
            }
        }
    }
    return false;
}

void LidarC1::ParseScanPacket(const uint8_t Packet[5]) {
    Parser.timeStamp = millis();
    Parser.quality = Packet[0] >> 2;
    // In the ESP32 Code it wraps angle but i dont think it can send an angle over 360 deg
    uint16_t rawAngle = (Packet[1] >> 1) | (Packet[2] << 7);
    uint16_t rawDistance = (Packet[3]) | (Packet[4] << 8);
    Parser.angle = rawAngle/64.0f;
    Parser.distance = (rawDistance & 0x7FFF)/4.0f;
    //float rad = Parser.angle*(pi/180.0f);
    //Parser.newX = Parser.distance*cosf(rad-(pi/2));
    //Parser.newY = Parser.distance*sinf(rad-(pi/2));

}
bool LidarC1::GetSingleScan() {
    if (Scanning) {
        if (!ReadScanPacket()) return false;
        m[0].timeStamp[0] = Parser.timeStamp;
        m[0].quality[0] = Parser.quality;
        m[0].angle[0] = Parser.angle;
        m[0].distance[0] = Parser.distance;
        //Measurements[0].NewXCoord[0] = Parser.newX;
        //Measurements[0].NewYCoord[0] = Parser.newY;
        return true;
        
    }
    return false;
}

bool LidarC1::GetFullScan(uint8_t NumofCycles) {
    for (uint8_t CycleIDX = 0; CycleIDX < NumofCycles; CycleIDX++) {
        uint32_t StartTime = 0;
        uint16_t IDX = 0;
        if (Scanning) {
            if (IDX < MAXPOINTS) {
                while (true) {
                    if (!ReadScanPacket()) return false;
                    if (Parser.startFlag) {
                        StartTime = millis();
                        m[CycleIDX].timeStamp[IDX] = Parser.timeStamp;
                        m[CycleIDX].quality[IDX] = Parser.quality;
                        m[CycleIDX].angle[IDX] = Parser.angle;
                        m[CycleIDX].distance[IDX] = Parser.distance;
                        //Measurements[CycleIDX].NewXCoord[IDX] = Parser.newX;
                        //Measurements[CycleIDX].NewYCoord[IDX] = Parser.newY;
                        IDX++;
                        break;
                    }
                }
                while (true) {
                    if (!ReadScanPacket()) return false;
                    if (!Parser.startFlag) {
                        m[CycleIDX].timeStamp[IDX] = Parser.timeStamp;
                        m[CycleIDX].quality[IDX] = Parser.quality;
                        m[CycleIDX].angle[IDX] = Parser.angle;
                        m[CycleIDX].distance[IDX] = Parser.distance;
                        //Measurements[CycleIDX].NewXCoord[IDX] = Parser.newX;
                        //Measurements[CycleIDX].NewYCoord[IDX] = Parser.newY;
                        IDX++;
                    } else {
                        m[CycleIDX].time = millis() - StartTime;
                        m[CycleIDX].count = IDX;
                        break;
                    }
                }
            }  
        } 
    } 
    return true;
}

MeasurementData GetMeasuremen() {
    return m[MAXCYCLES]
}

void LidarC1::serialClear() {
    while (serial.available() > 0) {
        serial.read();
    }
}
