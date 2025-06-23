#ifndef GSR_SENSOR_H
#define GSR_SENSOR_H

#include "SensorBase.h"
#include <Arduino.h>

class GSRSensor : public SensorBase {
public:
    // Constructor takes the analog pin the GSR sensor is connected to
    GSRSensor(uint8_t pin) : gsrPin(pin), initialized(false), filteredValue(0) {}

    bool begin() override;
    void update(HealthData &data) override;

private:
    uint8_t gsrPin;
    bool initialized;
    float filteredValue;
};

#endif // GSR_SENSOR_H
