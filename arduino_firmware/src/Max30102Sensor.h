#ifndef MAX30102_SENSOR_H
#define MAX30102_SENSOR_H

#include "SensorBase.h"
#include <Wire.h>

// I2C address of MAX30102 (7-bit address, usually 0x57 for MAX30102)
#define MAX30102_ADDR 0x57

class Max30102Sensor : public SensorBase {
public:
    Max30102Sensor() : lastIR(0), lastBeatTime(0), beatDetectedFlag(false),
                       ratesIndex(0), ratesCount(0) {
        // Initialize rate array to 0
        for (uint8_t i = 0; i < RATE_SIZE; ++i) { rates[i] = 0; }
        // Initialize SpO2 tracking variables
        resetCycleMeasurements();
    }

    bool begin() override;
    void update(HealthData &data) override;

    // After update(), this can be checked to see if a new beat was detected
    bool beatDetected;
    // If a beat was detected, this holds the last inter-beat interval in ms
    uint32_t getLastBeatInterval() const { return lastBeatInterval; }

private:
    // Internal state for beat detection and HR calculation
    static const uint8_t RATE_SIZE = 4;      // number of readings to average for heartRate
    uint8_t rates[RATE_SIZE];               // rolling buffer of recent heart rates
    uint8_t ratesIndex;                     // current index in rates buffer
    uint8_t ratesCount;                     // number of valid entries in rates (<= RATE_SIZE)
    long lastIR;                            // last IR value (for beat detection)
    uint32_t lastBeatTime;                  // time of last beat (ms)
    uint32_t lastBeatInterval;              // last beat-to-beat interval (ms)
    bool beatDetectedFlag;                  // internal flag for beat detection

    // Variables for SpO2 calculation across a pulse cycle
    uint32_t irMin, irMax;
    uint32_t redMin, redMax;
    bool firstCycle;  // indicates first cycle (to skip SpO2 calc on first beat)

    // Reset min/max for a new pulse cycle
    void resetCycleMeasurements() {
        irMin = 0xFFFFFFFF;
        irMax = 0;
        redMin = 0xFFFFFFFF;
        redMax = 0;
    }
};

#endif // MAX30102_SENSOR_H
