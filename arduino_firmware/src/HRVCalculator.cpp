#include "HRVCalculator.h"

void HRVCalculator::addInterval(uint32_t intervalMs) {
    // Add interval to circular buffer
    intervals[index] = intervalMs;
    if (count < HRV_BUFFER_SIZE) {
        ++count;
    }
    index = (index + 1) % HRV_BUFFER_SIZE;
}

float HRVCalculator::computeSDNN() const {
    if (count == 0) return 0.0f;
    uint8_t N = count;
    // Calculate mean of intervals
    double sum = 0.0;
    for (uint8_t i = 0; i < N; ++i) {
        sum += intervals[i];
    }
    double mean = sum / N;
    // Calculate variance
    double sumSqDiff = 0.0;
    for (uint8_t i = 0; i < N; ++i) {
        double diff = intervals[i] - mean;
        sumSqDiff += diff * diff;
    }
    double variance = sumSqDiff / N;
    float sdnn = sqrt(variance);
    return sdnn;
}

uint32_t HRVCalculator::getLastInterval() const {
    if (count == 0) return 0;
    uint8_t lastIndex = (index + HRV_BUFFER_SIZE - 1) % HRV_BUFFER_SIZE;
    return intervals[lastIndex];
}

uint32_t HRVCalculator::getPrevInterval() const {
    if (count < 2) return 0;
    uint8_t prevIndex = (index + HRV_BUFFER_SIZE - 2) % HRV_BUFFER_SIZE;
    return intervals[prevIndex];
}
