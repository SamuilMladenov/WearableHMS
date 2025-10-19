#pragma once

#include <Wire.h>
#include <Arduino.h>

class MLX90614 {
public:
    MLX90614();
    bool begin(uint8_t address = 0x5A);
    float readTemperature();
    float readAmbientTemperature();
    void printDiagnostics();
    
private:
    uint8_t _address;
    bool _initialized;
    
    bool readRegister16(uint8_t reg, uint16_t &value);
};