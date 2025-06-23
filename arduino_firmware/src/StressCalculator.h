#ifndef STRESS_CALCULATOR_H
#define STRESS_CALCULATOR_H

#include "HealthData.h"

class StressCalculator {
public:
    // Compute stress level (0-100) from current health data
    float computeStress(const HealthData &data) {
        // Normalize HRV: assume HRV (SDNN) 50ms or above is relaxed (0 stress contribution),
        // and HRV 0ms is extremely stressed (100). Clamp values outside range.
        float hrvComponent;
        if (data.hrv >= 50.0f) hrvComponent = 0.0f;
        else {
            hrvComponent = (50.0f - data.hrv) / 50.0f * 100.0f;
            if (hrvComponent < 0.0f) hrvComponent = 0.0f;
            if (hrvComponent > 100.0f) hrvComponent = 100.0f;
        }
        // Normalize heart rate: assume 60 BPM = 0 stress, 100 BPM or more = 100 stress (linear in between)
        float hrComponent;
        if (data.heartRate <= 60.0f) hrComponent = 0.0f;
        else if (data.heartRate >= 100.0f) hrComponent = 100.0f;
        else hrComponent = ((data.heartRate - 60.0f) / 40.0f) * 100.0f;
        if (hrComponent < 0.0f) hrComponent = 0.0f;
        if (hrComponent > 100.0f) hrComponent = 100.0f;
        // Normalize GSR: our data.gsr is 0-1023. Assume 0 = dry (0 stress), 1023 = max sweat (100 stress)
        float gsrComponent = (data.gsr / 1023.0f) * 100.0f;
        if (gsrComponent < 0.0f) gsrComponent = 0.0f;
        if (gsrComponent > 100.0f) gsrComponent = 100.0f;

        // Weight the components. HRV is a strong indicator (inverse relation), GSR and HR moderate.
        float stressScore = 0.5f * hrvComponent + 0.25f * hrComponent + 0.25f * gsrComponent;
        if (stressScore < 0.0f) stressScore = 0.0f;
        if (stressScore > 100.0f) stressScore = 100.0f;
        return stressScore;
    }
};

#endif // STRESS_CALCULATOR_H
