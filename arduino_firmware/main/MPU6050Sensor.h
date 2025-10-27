#pragma once
#include <Arduino.h>
#include <Wire.h>

class MPU6050 {
public:
    MPU6050();

    // Initialize device
    bool begin(uint8_t address = 0x68);

    // Read raw motion values (g and deg/s)
    bool readMotion(float &accelX, float &accelY, float &accelZ,
                    float &gyroX, float &gyroY, float &gyroZ);

    // Print one-shot diagnostics for debugging
    void printDiagnostics();

    // Check if the device is currently at rest
    bool isAtRest();

    // Call periodically (e.g., every 100 ms) to update internal motion stats
    void updateMotionState();

    // Optional accessors
    float getAccelRMS() const { return _accRMS; }
    float getGyroRMS() const { return _gyroRMS; }

private:
    uint8_t _address;
    bool _initialized;

    // Internal accumulators for rest detection
    float _accRMS;
    float _gyroRMS;
    unsigned long _lastUpdate;

    // Thresholds for rest detection
    const float ACC_REST_THRESHOLD = 0.015f;   // g
    const float GYRO_REST_THRESHOLD = 1.5f;   // deg/s
    const unsigned long REST_STABLE_TIME = 1500; // ms

    bool _currentlyRest;
    unsigned long _restStartTime;

    // Low-level register helpers
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t &value);
};
