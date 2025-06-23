#ifndef ARRHYTHMIA_DETECTOR_H
#define ARRHYTHMIA_DETECTOR_H

#include "HRVCalculator.h"

class ArrhythmiaDetector {
public:
    bool checkForArrhythmia(float currentBPM, const HRVCalculator &hrvCalc) {
        bool arrhythmia = false;
        // Check for heart rate out of normal range (tachycardia/bradycardia)
        if (currentBPM > 100.0f || currentBPM < 60.0f) {
            arrhythmia = true; // heart rate outside 60-100 BPM:contentReference[oaicite:27]{index=27}
        }
        // Check for irregular interval jump between last two beats
        uint32_t lastInt = hrvCalc.getLastInterval();
        uint32_t prevInt = hrvCalc.getPrevInterval();
        if (lastInt > 0 && prevInt > 0) {
            if (lastInt > 1.5 * prevInt || lastInt * 1.5 < prevInt) {
                // If one interval is 50% larger or smaller than the previous, flag irregular
                arrhythmia = true;
            }
        }
        // (Additional criteria like overall HRV too high or low could be added if needed)
        return arrhythmia;
    }
};

#endif // ARRHYTHMIA_DETECTOR_H
