#include "MPU6050Sensor.h"
#include <math.h>

bool MPU6050Sensor::begin() {
    Wire.beginTransmission(MPU6050_ADDR);
    if (Wire.endTransmission() != 0) {
        return false; // sensor not responding
    }
    // Wake up the MPU6050 (exit sleep mode)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0x00);  // set to zero (wakes up the MPU-6050)
    Wire.endTransmission();
    // Optionally, set accelerometer range, bandwidth, etc. (defaults are fine: ±2g, 1kHz)
    initialized = true;
    // Read initial values to set baseline
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // starting register for accel data
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6);
    if (Wire.available() == 6) {
        // Combine high and low bytes for each axis
        lastAx = (Wire.read() << 8) | Wire.read();
        lastAy = (Wire.read() << 8) | Wire.read();
        lastAz = (Wire.read() << 8) | Wire.read();
    }
    tremorCounter = 0;
    return true;
}

void MPU6050Sensor::update(HealthData &data) {
    if (!initialized) return;
    // Read accelerometer data (X, Y, Z – 16 bits each)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6);
    if (Wire.available() < 6) {
        return;
    }
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();

    // Compute difference from last reading
    long diffX = ax - lastAx;
    long diffY = ay - lastAy;
    long diffZ = az - lastAz;
    // Magnitude of difference vector
    float diffMag = sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);

    // Update last values for next iteration
    lastAx = ax;
    lastAy = ay;
    lastAz = az;

    // Thresholds (tuned empirically for sensitivity)
    const float TREMOR_DIFF_THRESHOLD = 500.0f;   // small movements (around ~0.03g if 16384=1g)
    const float MOTION_DIFF_THRESHOLD = 3000.0f;  // larger movements (0.18g and above)
    
    // Check for significant motion vs small tremor-like vibrations
    if (diffMag > MOTION_DIFF_THRESHOLD) {
        // Large movement detected – reset tremor counter (likely deliberate motion)
        tremorCounter = 0;
        data.tremorDetected = false;
    } else if (diffMag > TREMOR_DIFF_THRESHOLD) {
        // Small rapid change – potential tremor event
        if (tremorCounter < 255) tremorCounter++;
    } else {
        // No significant change this cycle
        if (tremorCounter > 0) tremorCounter--;
    }

    // Decide if tremor is sustained
    if (tremorCounter > 5) {
        data.tremorDetected = true;
    } else if (tremorCounter == 0) {
        data.tremorDetected = false;
    }
}
