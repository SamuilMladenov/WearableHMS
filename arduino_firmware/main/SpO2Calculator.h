#pragma once

#include <Arduino.h>
#include <vector>

struct SpO2Sample {
    unsigned long timestamp;
    uint16_t redValue;
    uint16_t irValue;
};

class SpO2Calculator {
public:
    SpO2Calculator();
    
    // Add a sample pair (RED and IR)
    void addSample(unsigned long timestamp, uint16_t redValue, uint16_t irValue);
    
    // Calculate SpO2 from collected samples
    float calculateSpO2();
    
    // Clear all samples
    void clearSamples();
    
    // Get number of samples collected
    size_t getSampleCount() const;
    
    // Get the calculated SpO2
    float getSpO2() const;
    
    // Check if we have a valid SpO2
    bool hasValidSpO2() const;
    
    // Print sample statistics
    void printSampleStats() const;

private:
    std::vector<SpO2Sample> _samples;
    float _spo2;
    bool _hasValidSpO2;
    float _ratioOfRatios;
    
    // Calculate AC and DC components
    void calculateACDC(const std::vector<uint16_t>& signal, float& ac, float& dc);
    
    // Apply calibration curve (empirical formula)
    float applyCalibration(float ratio);
};