#include "GSRSensor.h"

bool GSRSensor::begin() {
    // Configure the analog pin if necessary (Nano 33 BLE has default 10-bit ADC)
    pinMode(gsrPin, INPUT);
    // We will do a simple calibration: read a baseline value at start (assuming calm state)
    int baseline = analogRead(gsrPin);
    filteredValue = (float) baseline;
    initialized = true;
    return true;
}

void GSRSensor::update(HealthData &data) {
    if (!initialized) return;
    int raw = analogRead(gsrPin);
    // Simple exponential moving average filter to smooth out noise
    filteredValue = 0.8f * filteredValue + 0.2f * raw;
    data.gsr = filteredValue;
    // (The raw value 0-1023 is stored. Conversion to actual conductance or microSiemens 
    // would require knowing the sensor's circuit. Here we use the raw value as an index.)
    // Note: GSR is typically a relative measure, so we don't need to convert to absolute units.
    // And also the stress level calculator normalizes the level based on the GSR range.
}
