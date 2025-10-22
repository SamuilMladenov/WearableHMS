#include "SpO2Calculator.h"

void SpO2Calculator::addSample(uint16_t red, uint16_t ir) {
    redSamples.push_back(red);
    irSamples.push_back(ir);
}

float SpO2Calculator::computeDC(const std::vector<uint16_t>& samples) {
    if (samples.empty()) return 0;
    float sum = 0;
    for (uint16_t s : samples) sum += s;
    return sum / samples.size();
}

float SpO2Calculator::computeAC(const std::vector<uint16_t>& samples, float dc) {
    float maxVal = 0, minVal = 65535;
    for (uint16_t s : samples) {
        if (s > maxVal) maxVal = s;
        if (s < minVal) minVal = s;
    }
    return (maxVal - minVal) / 2.0;
}

float SpO2Calculator::calculateSpO2() {
    if (redSamples.size() < 50 || irSamples.size() < 50) return 0.0;

    float dcRed = computeDC(redSamples);
    float dcIR  = computeDC(irSamples);
    float acRed = computeAC(redSamples, dcRed);
    float acIR  = computeAC(irSamples, dcIR);

    if (acRed == 0 || acIR == 0) return 0.0;

    // Ratio-of-ratios formula
    float R = (acRed / dcRed) / (acIR / dcIR);
    float spo2 = 104.0f - 17.0f * R;
    spo2 = constrain(spo2, 70.0f, 100.0f);

    return spo2;
}
