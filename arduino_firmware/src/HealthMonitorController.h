#ifndef HEALTH_MONITOR_CONTROLLER_H
#define HEALTH_MONITOR_CONTROLLER_H

#include "Max30102Sensor.h"
#include "GSRSensor.h"
#include "MLX90614Sensor.h"
#include "MPU6050Sensor.h"
#include "HRVCalculator.h"
#include "StressCalculator.h"
#include "ArrhythmiaDetector.h"
#include "BLECommunicator.h"

class HealthMonitorController {
public:
    HealthMonitorController() :
        heartSensor(), 
        gsrSensor(A0),           // assuming GSR is connected to A0
        tempSensor(),
        motionSensor()
    {}

    bool begin();
    void update();

private:
    // Sensor objects
    Max30102Sensor heartSensor;
    GSRSensor gsrSensor;
    MLX90614Sensor tempSensor;
    MPU6050Sensor motionSensor;
    // Calculation and analysis objects
    HRVCalculator hrvCalculator;
    StressCalculator stressCalculator;
    ArrhythmiaDetector arrDetector;
    // BLE communicator
    BLECommunicator bleComm;
    // Data structure to hold current readings
    HealthData currentData;
    // Timing for BLE transmissions
    const unsigned long SEND_INTERVAL_MS = 1000;
    unsigned long lastSendTime = 0;
};

#endif // HEALTH_MONITOR_CONTROLLER_H
