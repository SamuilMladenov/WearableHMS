#pragma once
#include <Arduino.h>
#include <vector>

class SpO2Calculator {
public:
    void addSample(uint16_t red, uint16_t ir);
    float calculateSpO2();
    void clear() { redSamples.clear(); irSamples.clear(); }


private:
    std::vector<uint16_t> redSamples;
    std::vector<uint16_t> irSamples;
    float computeDC(const std::vector<uint16_t>& samples);
    float computeAC(const std::vector<uint16_t>& samples, float dc);
};
