#include "BLECommunicator.h"

bool BLECommunicator::begin() {
    if (!BLE.begin()) {
        return false;
    }
    BLE.setDeviceName("HealthMonitor");
    BLE.setLocalName("HealthMonitor");
    // Set up custom service and characteristic
    BLE.setAdvertisedService(healthService);
    healthService.addCharacteristic(dataChar);
    BLE.addService(healthService);
    // Initial value for the characteristic (empty JSON)
    dataChar.writeValue("{}");
    // Start advertising
    BLE.advertise();
    return true;
}

void BLECommunicator::sendData(const HealthData &data) {
    // Only send if a central device is connected and subscribed
    BLEDevice central = BLE.central();
    if (!central || !central.connected()) {
        return;
    }
    // Format the health data as a JSON string
    char jsonBuffer[100];
    snprintf(jsonBuffer, sizeof(jsonBuffer),
        "{"
        "\"hr\":%.0f,"           // heartRate as integer
        "\"spo2\":%.1f,"         // SpO2 with one decimal
        "\"temp\":%.2f,"         // temperature with two decimals
        "\"gsr\":%.0f,"          // GSR raw
        "\"hrv\":%.1f,"          // HRV (ms) with one decimal
        "\"stress\":%.0f,"       // stress level as integer
        "\"arr\":%s,"            // arrhythmia flag (true/false)
        "\"tremor\":%s"          // tremor flag (true/false)
        "}",
        data.heartRate,
        data.spo2,
        data.bodyTemp,
        data.gsr,
        data.hrv,
        data.stressLevel,
        data.arrhythmiaDetected ? "true" : "false",
        data.tremorDetected ? "true" : "false"
    );
    // Update characteristic value and notify central
    dataChar.writeValue((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
}
