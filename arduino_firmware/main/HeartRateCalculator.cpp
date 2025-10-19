#include "HeartRateCalculator.h"

HeartRateCalculator::HeartRateCalculator(uint16_t threshold) 
    : _threshold(threshold), 
      _bpm(0), 
      _hasValidBPM(false),
      _peakCount(0) {
    _samples.reserve(2000); // Reserve space for ~20 seconds at 100Hz
}

void HeartRateCalculator::addSample(unsigned long timestamp, uint16_t value) {
    _samples.push_back({timestamp, value});
}

void HeartRateCalculator::clearSamples() {
    _samples.clear();
    _hasValidBPM = false;
    _peakCount = 0;
}

size_t HeartRateCalculator::getSampleCount() const {
    return _samples.size();
}

float HeartRateCalculator::getBPM() const {
    return _bpm;
}

bool HeartRateCalculator::hasValidBPM() const {
    return _hasValidBPM;
}

void HeartRateCalculator::setThreshold(uint16_t threshold) {
    _threshold = threshold;
}

uint16_t HeartRateCalculator::getPeakCount() const {
    return _peakCount;
}

uint16_t HeartRateCalculator::getSampleValue(size_t index) const {
    if (index >= _samples.size()) return 0;
    return _samples[index].value;
}

void HeartRateCalculator::printSampleStats() const {
    if (_samples.empty()) {
        Serial.println("  No samples collected");
        return;
    }
    
    Serial.println("  Sample values (first 20):");
    for (size_t i = 0; i < 20 && i < _samples.size(); i++) {
        if (i % 5 == 0) Serial.print("    ");
        Serial.print(_samples[i].value);
        Serial.print(" ");
        if ((i + 1) % 5 == 0) Serial.println();
    }
    if (_samples.size() % 5 != 0) Serial.println();
}

std::vector<unsigned long> HeartRateCalculator::findPeaks() {
    std::vector<unsigned long> peaks;
    
    if (_samples.size() < 3) {
        return peaks;
    }
    
    // Simple peak detection - look for rising edge crossing threshold
    for (size_t i = 1; i < _samples.size(); i++) {
        if (_samples[i].value > _threshold && _samples[i-1].value <= _threshold) {
            peaks.push_back(_samples[i].timestamp);
        }
    }
    
    return peaks;
}

std::vector<unsigned long> HeartRateCalculator::findPeaksAdaptive(uint16_t threshold) {
    std::vector<unsigned long> peaks;
    
    if (_samples.size() < 5) {
        return peaks;
    }
    
    // Look for local maxima above threshold
    for (size_t i = 2; i < _samples.size() - 2; i++) {
        uint16_t val = _samples[i].value;
        
        // Check if this is a local maximum
        if (val > threshold &&
            val > _samples[i-1].value && 
            val > _samples[i-2].value &&
            val > _samples[i+1].value && 
            val > _samples[i+2].value) {
            
            // Make sure we don't add peaks too close together (minimum 200ms apart)
            if (peaks.empty() || (_samples[i].timestamp - peaks.back() > 200)) {
                peaks.push_back(_samples[i].timestamp);
            }
        }
    }
    
    return peaks;
}

float HeartRateCalculator::calculateBPM() {
    _hasValidBPM = false;
    _peakCount = 0;
    
    if (_samples.size() < 100) {
        Serial.println("  Not enough samples");
        return 0.0; // Not enough data
    }
    
    // Calculate statistics on the signal
    uint16_t minVal = 65535, maxVal = 0;
    unsigned long sumVal = 0;
    
    for (size_t i = 0; i < _samples.size(); i++) {
        if (_samples[i].value < minVal) minVal = _samples[i].value;
        if (_samples[i].value > maxVal) maxVal = _samples[i].value;
        sumVal += _samples[i].value;
    }
    
    float avgVal = (float)sumVal / _samples.size();
    uint16_t signalRange = maxVal - minVal;
    
    Serial.print("  Signal: min=");
    Serial.print(minVal);
    Serial.print(" max=");
    Serial.print(maxVal);
    Serial.print(" avg=");
    Serial.print(avgVal);
    Serial.print(" range=");
    Serial.println(signalRange);
    
    // Check if signal has enough variation
    if (signalRange < 50) {
        Serial.println("  Signal range too small - sensor might not be touching skin");
        return 0.0;
    }
    
    // Adaptive threshold: use percentage of range above average
    uint16_t adaptiveThreshold = avgVal + (signalRange * 0.3); // 30% above average
    Serial.print("  Using adaptive threshold: ");
    Serial.println(adaptiveThreshold);
    
    std::vector<unsigned long> peaks = findPeaksAdaptive(adaptiveThreshold);
    _peakCount = peaks.size();
    
    Serial.print("  Peaks found: ");
    Serial.println(_peakCount);
    
    if (peaks.size() < 3) {
        return 0.0; // Need at least 3 peaks
    }
    
    // Calculate intervals between peaks
    std::vector<unsigned long> intervals;
    for (size_t i = 1; i < peaks.size(); i++) {
        unsigned long interval = peaks[i] - peaks[i-1];
        
        // Filter out unrealistic intervals (300ms - 2000ms = 30-200 BPM)
        if (interval >= 300 && interval <= 2000) {
            intervals.push_back(interval);
        }
    }
    
    if (intervals.size() < 2) {
        Serial.println("  Not enough valid intervals");
        return 0.0; // Not enough valid intervals
    }
    
    // Calculate average interval
    unsigned long sum = 0;
    for (unsigned long interval : intervals) {
        sum += interval;
    }
    float avgInterval = (float)sum / intervals.size();
    
    // Convert to BPM
    _bpm = 60000.0 / avgInterval;
    
    // Sanity check
    if (_bpm < 30 || _bpm > 200) {
        Serial.print("  BPM out of range: ");
        Serial.println(_bpm);
        return 0.0;
    }
    
    _hasValidBPM = true;
    
    return _bpm;
}