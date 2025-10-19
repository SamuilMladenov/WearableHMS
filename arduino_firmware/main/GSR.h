#pragma once

#include <Arduino.h>

class GSR {
public:
    GSR(uint8_t pin = A0);
    bool begin(uint8_t pin = A0);
    float readGSR();
    void printDiagnostics();
    
private:
    uint8_t _pin;
    bool _initialized;
};