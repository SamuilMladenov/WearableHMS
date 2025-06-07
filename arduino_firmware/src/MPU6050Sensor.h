#ifndef MPU6050_SENSOR_H
#define MPU6050_SENSOR_H

#include "SensorBase.h"
#include <Wire.h>

#define MPU6050_ADDR 0x68  // I2C address of MPU-6050 (when AD0 = LOW)

class MPU6050Sensor : public SensorBase {
public:
    MPU6050Sensor() : lastAx(0), lastAy(0), lastAz(0), initialized(false),
                      tremorCounter(0) {}

    bool begin() override;
    void update(HealthData &data) override;

private:
    // Last accelerometer readings
    int16_t lastAx, lastAy, lastAz;
    bool initialized;
    // Counter for detecting sustained tremor vibration
    uint8_t tremorCounter;
};

#endif // MPU6050_SENSOR_H
