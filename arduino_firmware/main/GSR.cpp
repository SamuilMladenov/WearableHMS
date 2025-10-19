#include "GSR.h"

GSR::GSR(uint8_t pin) : _pin(pin), _initialized(false) {
}

bool GSR::begin(uint8_t pin) {
    _pin = pin;
    pinMode(_pin, INPUT);
    _initialized = true;
    return true;
}

float GSR::readGSR() {
    if (!_initialized) return -1.0;
    
    // Read analog value
    int rawValue = analogRead(_pin);
    
    // Simple validation - check if we're getting actual readings
    if (rawValue == 0 || rawValue == 1023) {
        // Might be disconnected or shorted
        return rawValue; // Return raw value for debugging
    }
    
    // Convert to voltage (adjust for 3.3V or 5V system)
    #ifdef ARDUINO_ARCH_ESP32
    float voltage = (rawValue / 4095.0) * 3.3; // ESP32 is 12-bit, 3.3V
    #else
    float voltage = (rawValue / 1023.0) * 5.0; // Arduino is 10-bit, 5V
    #endif
    
    // If using a voltage divider with 10k resistor:
    // GSR = (Vcc - V) * R / V
    #ifdef ARDUINO_ARCH_ESP32
    float vcc = 3.3;
    #else
    float vcc = 5.0;
    #endif
    
    if (voltage <= 0.01) return 999999.0; // Avoid division by zero
    
    float gsrResistance = ((vcc - voltage) * 10000.0) / voltage;
    
    return gsrResistance; // Returns resistance in ohms
}

void GSR::printDiagnostics() {
    if (!_initialized) {
        Serial.println("GSR: Not initialized");
        return;
    }
    
    float gsr = readGSR();
    Serial.print("GSR: ");
    Serial.print(gsr);
    Serial.println(" ohms");
}