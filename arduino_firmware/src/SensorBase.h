#ifndef SENSOR_BASE_H
#define SENSOR_BASE_H

#include "HealthData.h"

// Abstract base class for all sensors
class SensorBase {
public:
    // Initialize the sensor (returns true if successful)
    virtual bool begin() = 0;
    // Update the sensor reading and store it in the provided HealthData
    virtual void update(HealthData &data) = 0;
    // Virtual destructor
    virtual ~SensorBase() {}
};

#endif // SENSOR_BASE_H
