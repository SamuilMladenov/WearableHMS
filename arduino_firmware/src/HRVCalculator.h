#ifndef HRV_CALCULATOR_H
#define HRV_CALCULATOR_H

#include <math.h>
#include <stdint.h>

#define HRV_BUFFER_SIZE 30  // number of recent intervals to store for HRV calculation

class HRVCalculator {
public:
    HRVCalculator() : count(0), index(0) {}

    // Add a new inter-beat interval (in milliseconds)
    void addInterval(uint32_t intervalMs);

    // Compute SDNN (standard deviation of intervals in the buffer) in milliseconds
    float computeSDNN() const;

    // Optional: retrieve last and previous intervals (for arrhythmia analysis)
    uint32_t getLastInterval() const;
    uint32_t getPrevInterval() const;
    uint8_t getCount() const { return count; }

private:
    uint32_t intervals[HRV_BUFFER_SIZE];
    uint8_t count;
    uint8_t index;
};

#endif // HRV_CALCULATOR_H
