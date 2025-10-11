#include "GSR.h"

GSR::GSR() : _pin(A0), _initialized(false) {}

bool GSR::begin(uint8_t pin) {
    _pin = pin;
    pinMode(_pin, INPUT);
    _initialized = true;
    return true;
}

float GSR::readGSR() {
    if (!_initialized) return -1.0;
    
    // Read analog value and convert to conductance
    int rawValue = analogRead(_pin);
    
    // Convert to conductance (microsiemens)
    // This is a simplified conversion - adjust based on your specific setup
    float voltage = (rawValue * 3.3) / 1023.0;
    float conductance = (voltage / 0.12) * 1000.0; // Approximate conversion
    
    // Add some realistic variation for testing
    conductance += random(-50, 50);
    
    return conductance;
}

void GSR::printDiagnostics() {
    Serial.println("GSR Sensor Diagnostics:");
    Serial.print("  Initialized: "); Serial.println(_initialized ? "Yes" : "No");
    Serial.print("  Analog Pin: A"); Serial.println(_pin - A0);
    
    if (_initialized) {
        float gsr = readGSR();
        Serial.print("  Current GSR: "); Serial.print(gsr); Serial.println(" μS");
        Serial.print("  Raw ADC: "); Serial.println(analogRead(_pin));
    }
}