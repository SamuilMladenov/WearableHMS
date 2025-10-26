#pragma once

#include <Arduino.h>
#include <vector>

struct HeartSample {
    unsigned long timestamp;
    uint16_t value;
};

class HeartRateCalculator {
public:
    HeartRateCalculator(uint16_t threshold = 500);
    
    // Add a sample to the collection
    void addSample(unsigned long timestamp, uint16_t value);
    
    // Calculate BPM from collected samples
    float calculateBPM();
    
    // Clear all samples
    void clearSamples();
    
    // Get number of samples collected
    size_t getSampleCount() const;
    
    // Get the calculated BPM
    float getBPM() const;
    
    // Check if we have a valid BPM
    bool hasValidBPM() const;
    
    // Adjust threshold if needed
    void setThreshold(uint16_t threshold);
    
    // Get peak count from last calculation
    uint16_t getPeakCount() const;
    
    // Get sample value at index (for debugging)
    uint16_t getSampleValue(size_t index) const;
    
    // Print sample statistics
    void printSampleStats() const;

private:
    std::vector<HeartSample> _samples;
    std::vector<unsigned long> _recentIntervals;  // For adaptive refractory
    uint16_t _threshold;
    float _bpm;
    bool _hasValidBPM;
    uint16_t _peakCount;
    
    // Find peaks in the sample data
    std::vector<unsigned long> findPeaks();
    
    // Find peaks with adaptive threshold
    std::vector<unsigned long> findPeaksAdaptive(uint16_t threshold);
};