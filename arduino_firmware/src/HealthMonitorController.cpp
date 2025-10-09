#include "HealthMonitorController.h"
#include <Arduino.h>

bool HealthMonitorController::begin() {
    bool ok = true;
    // Initialize all sensors
    if (!heartSensor.begin()) {
        Serial.println("ERROR: Hearth sensor not found!");
        ok = false;
    }
    if (!gsrSensor.begin()) {
        Serial.println("ERROR: GSR sensor init failed!");
        ok = false;
    }
    if (!tempSensor.begin()) {
        Serial.println("ERROR: MLX90614 not found!");
        ok = false;
    }
    if (!motionSensor.begin()) {
        Serial.println("ERROR: MPU6050 not found!");
        ok = false
    }
    // Initialize BLE communication
    if (!bleComm.begin()) {
        Serial.println("ERROR: BLE initialization failed!");
        ok = false;
    } else {
        Serial.println("BLE device is now advertising...");
    }
    return ok;
}

void HealthMonitorController::update() {
    // Update each sensor; each will update relevant fields in currentData
    heartSensor.update(currentData);
    gsrSensor.update(currentData);
    tempSensor.update(currentData);
    motionSensor.update(currentData);

    // If a heartbeat was detected in this update, add the interval to HRV calculator
    if (heartSensor.beatDetected) {
        uint32_t interval = heartSensor.getLastBeatInterval();
        if (interval > 0) {
            hrvCalculator.addInterval(interval);
        }
        heartSensor.beatDetected = false; // reset flag
    }
    // Update activity status based on motion sensor
    float ax = motionSensor.getLastAx();
    float ay = motionSensor.getLastAy();
    float az = motionSensor.getLastAz();

    activityDetector.update(ax, ay, az);


    // Decide if it's time to send an update (e.g., every 1000 ms)
    unsigned long now = millis();
    if (now - lastSendTime >= SEND_INTERVAL_MS) {
        // Compute derived metrics
        currentData.hrv = hrvCalculator.computeSDNN();
        currentData.stressLevel = stressCalculator.computeStress(currentData);
        currentData.arrhythmiaDetected = arrDetector.checkForArrhythmia(currentData.heartRate, hrvCalculator);
        currentData.active = activityDetector.isActive();
        // (currentData.tremorDetected is updated in motionSensor.update())
        // Send data via BLE
        bleComm.sendData(currentData);
        lastSendTime = now;
    }
}
