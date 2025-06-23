#include <Arduino.h>
#include "HealthMonitorController.h"

HealthMonitorController controller;

void setup() {
    Serial.begin(115200);
    while (!Serial) { /* wait for serial port to open, for debugging */ }

    Serial.println("Initializing Health Monitor...");
    bool success = controller.begin();
    if (!success) {
        Serial.println("Initialization failed. Check sensor connections.");
    } else {
        Serial.println("Initialization complete. Starting monitoring...");
    }
}

void loop() {
    controller.update();
    // Small delay or yield can be added to reduce CPU usage if needed
    delay(5); // adjust as necessary
}
