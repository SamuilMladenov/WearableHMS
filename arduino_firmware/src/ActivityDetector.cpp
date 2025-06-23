#include "ActivityDetector.h"
#include <math.h>

ActivityDetector::ActivityDetector()
    : threshold(1.2f * 9.81f), active(false) {} // ~1.2g in m/s^2

void ActivityDetector::update(float ax, float ay, float az) {
    float magnitude = sqrt(ax * ax + ay * ay + az * az);
    active = (magnitude > threshold);
}

bool ActivityDetector::isActive() const {
    return active;
}
