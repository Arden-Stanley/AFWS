#include <Arduino.h>
#include "LidarC1.h"

#define Debug

// Notes
// Make look up table for commands stored in progmem
// Add connection failure timeouts
// Double Check bitwise operations




#define StartByte 0xA5

// Command bytes
#define STOPCMD 0x25
#define RESETCMD 0x40
#define SCANCMD 0x20
#define GETHEALTHCMD 0x52
#define MOTORSPEEDCMD 0xA8

// DataType Bytes
#define SCANDATATYPE 0x81
#define HEALTHDATATYPE 0x04

// Delay times
#define HEALTHDELAY 20
#define RESETDELAY 2000
#define SCANDELAY 20
#define STOPDELAY 20
#define MOTORDELAY 20



void LidarC1::begin(int BaudRate) {
    Serial1.begin(BaudRate); // Need to figure out what Serial pins to use
}


void LidarC1::GetHealth() {
    if (millis() > TCMD) {
        HealthPacket[0] = {StartByte};
        HealthPacket[1] = GETHEALTHCMD;
        Serial1.write(HealthPacket, sizeof(HealthPacket));
        TCMD = millis() + HEALTHDELAY;
    }

}
// Stop Command no return
void LidarC1::Stop() {
    if (millis() > TCMD) {
        StopPacket[0] = StartByte;
        StopPacket[1] = STOPCMD;
        Serial1.write(StopPacket, sizeof(StopPacket));
        TCMD = millis() + STOPDELAY;
    }
}
// Reset Command no return
void LidarC1::Reset() {
    if (millis() > TCMD) {
        ResetPacket[0] = StartByte;
        ResetPacket[1] = RESETCMD;
        Serial1.write(ResetPacket, sizeof(ResetPacket));
        TCMD = millis() + RESETDELAY;
    }
}
// Scan command 5 byte return constantly until stopped (im pretty sure)
void LidarC1::StartScan() {
    if (millis() > TCMD) {
        ScanTXPacket[0] = StartByte;
        ScanTXPacket[1] = SCANCMD;
        Serial1.write(ScanTXPacket, sizeof(ScanTXPacket));
        TCMD = millis() + SCANDELAY;
    }
}
// Set Motor Speed (16bit integer))
void LidarC1::MotorSpeed(uint16_t RPM) {
    if (millis() > TCMD) {
        SpeedPacket[0] = StartByte;
        SpeedPacket[1] = MOTORSPEEDCMD;
        SpeedPacket[2] = RPM >> 8 & 0xFF;
        SpeedPacket[3] = RPM & 0xFF;
        SpeedPacket[4] = 1;
        Serial1.write(SpeedPacket,sizeof(SpeedPacket));
        TCMD = millis() + MOTORDELAY;
    }
}

void LidarC1::ReadScan() {
    while (Serial1.available()) {
        ReadByte = Serial1.read();
        switch (ScanState) {
            case 0: // Searching for Packet Start
                if (ReadByte == StartByte) {
                    i = 0;
                    Buffer[i++] = ReadByte;
                    ScanState = 1;
                }
                break;
            case 1: // Get Descriptor
                Buffer[i++] = ReadByte;
                if (Buffer[1] != 0x5A) { 
                    ScanState = 0;
                }
                if (i == 7) {
                    ParseDescriptor(Buffer);
                    i = 0;
                    ScanState = 2;
                }
                break;
            case 2: // Recieve Data
                Buffer[i++] = ReadByte;
                if (i == DataLength) {
                    switch (DataType) {
                        case SCANDATATYPE: // Scan Data
                            ParseScanData(Buffer, DataLength);
                            i = 0;
                            break;
                        case HEALTHDATATYPE: // Health Data
                            ParseHealthData(Buffer, DataLength);
                            i = 0;
                            break;
                        default: 
                            Serial.println("Error: Unknown Data Type");
                            ScanState = 0;
                            i = 0;
                            break;
                    }
                }
        }
    }
}

// Descriptor Setup
// 7 Bytes Long
// Byte 1: Start Byte (0xA5)
// Byte 2: Descriptor Check Byte (0x5A)
// Byte 3: Data Length (little endian)
// Byte 4: Data Length
// Byte 5: Data Length
// Byte 6: Bits 8-2: Data Length Bits 2-0: Send Mode
// Byte 7: Data Type
void LidarC1::ParseDescriptor(uint8_t* Buffer) {
    memcpy(Descriptor.DescriptorPacket, Buffer, 7);
    Descriptor.DataLength = Descriptor.DescriptorPacket[2] | (Descriptor.DescriptorPacket[3] << 8) | (Descriptor.DescriptorPacket[4] << 16) | (Descriptor.DescriptorPacket[5] << 24) & 0x3FFFFFFF;
    Descriptor.SendMode = Descriptor.DescriptorPacket[5] & 0x03;
    Descriptor.DataType = Descriptor.DescriptorPacket[6];
}

// Scan Data Setup
// 5 Bytes Long
// 1bit: Start flag
// 2bit: Inverse Start flag
// 3-8bits: Quality
// 9 bit : Check Bit
// 10-16bits: Angle
// 17-24bits: Angle last 6 bits are decimal
// 25-32bits: Distance
// 33-40bits: Distance last 2 bits are decimal
void LidarC1::ParseScanData(uint8_t* Buffer, uint8_t DataLength) {
    CycleIDX = 0;
    if (CycleIDX <= 5) {
     switch (Buffer[0] & 0x01) {
        case 1: // Start index at beginning of new scan
            MIDX=0;
            ScanData.Quality[MIDX] = Buffer[0] >> 2;
            ScanData.Angle[MIDX] = ((Buffer[2] << 7) | (Buffer[1] >> 1))/64;
            ScanData.Distance[MIDX] = ((Buffer[4] << 8) | Buffer[3])/4;
            #ifdef Debug
            Serial.printf("Quality: %d, Angle: %d, Distance: %d",ScanData.Quality[MIDX],ScanData.Angle[MIDX],ScanData.Distance[MIDX]);
            CycleIDX++;
            #endif
            MIDX++;
            break;
        case 0: 
            ScanData.Quality[MIDX] = Buffer[0] >> 2;
            ScanData.Angle[MIDX] = ((Buffer[2] << 7) | (Buffer[1] >> 1))/64;
            ScanData.Distance[MIDX] =((Buffer[4] << 8) | Buffer[3])/4;
            #ifdef Debug
            Serial.printf("Quality: %d, Angle: %d, Distance: %d",ScanData.Quality[MIDX],ScanData.Angle[MIDX],ScanData.Distance[MIDX]);
            CycleIDX++;
            #endif
            MIDX++;
            break;
        }
    } else {
        LidarC1::Stop();
    }
    // Stops after 5 cycles
}

// Health Data Setup
// 3 Bytes long
// 1-8: Status
// 9-24: Error Code
void LidarC1::ParseHealthData(uint8_t* Buffer, uint8_t DataLength) {
    HealthData.Status = Buffer[0];
    HealthData.ErrorCode = Buffer[1] | (Buffer[2] << 8);
    #ifdef Debug
    switch (HealthData.Status) {
        case 0:
            Serial.println("Lidar Status: Good");
            break;
        case 1:
            Serial.println("Lidar Status: Warning");
            break;
        case 2:
            Serial.println("Lidar Status: Error");
            Serial.println("Error Code: " + String(HealthData.ErrorCode));
            break;
        default:
            Serial.println("Lidar Status: Unknown");
            break;
    }
    #endif
    ScanState = 0;
}