#pragma once
#include <Arduino.h>

class ActivityDetector {
public:
    ActivityDetector();

    // Update with new acceleration values
    void update(float ax, float ay, float az);

    // Returns true if the user is currently active
    bool isActive() const;

private:
    float threshold;  // acceleration magnitude threshold
    bool active;
};
