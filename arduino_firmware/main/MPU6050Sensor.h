#pragma once

#include <Wire.h>
#include <Arduino.h>

class MPU6050 {
public:
    MPU6050();
    bool begin(uint8_t address = 0x68);
    bool readMotion(float &accelX, float &accelY, float &accelZ, 
                    float &gyroX, float &gyroY, float &gyroZ);
    void printDiagnostics();
    
private:
    uint8_t _address;
    bool _initialized;
    
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t &value);
};