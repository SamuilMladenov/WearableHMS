#ifndef HEALTH_DATA_H
#define HEALTH_DATA_H

#include <Arduino.h>

struct HealthData {
    float heartRate;           // Heart rate in BPM
    float spo2;                // Blood oxygen level in %
    float bodyTemp;            // Body temperature in °C
    float gsr;                 // Skin conductance level (raw analog value)
    float hrv;                 // Heart rate variability (e.g., SDNN in ms)
    float stressLevel;         // Stress level (0-100 index)
    bool  arrhythmiaDetected;  // Arrhythmia flag
    bool  tremorDetected;      // Tremor detected flag
    bool active;              // Activity status (true if user is active)

    HealthData() : 
        heartRate(0.0f), spo2(0.0f), bodyTemp(0.0f), gsr(0.0f),
        hrv(0.0f), stressLevel(0.0f),
        arrhythmiaDetected(false), tremorDetected(false), active(false)
    {}
};

#endif // HEALTH_DATA_H
