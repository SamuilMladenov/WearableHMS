#ifndef MLX90614_SENSOR_H
#define MLX90614_SENSOR_H

#include "SensorBase.h"
#include <Wire.h>

#define MLX90614_I2C_ADDR 0x5A  // 7-bit I2C address of MLX90614

class MLX90614Sensor : public SensorBase {
public:
    bool begin() override;
    void update(HealthData &data) override;
};

#endif // MLX90614_SENSOR_H
