#include "HeartRateCalculator.h"
#include <algorithm>
#include <numeric>

HeartRateCalculator::HeartRateCalculator(uint16_t threshold)
    : _threshold(threshold),
      _bpm(0),
      _hasValidBPM(false),
      _peakCount(0) {
    _samples.reserve(2000); // ~20s at 100Hz
}

void HeartRateCalculator::addSample(unsigned long timestamp, uint16_t value) {
    _samples.push_back({timestamp, value});
}

void HeartRateCalculator::clearSamples() {
    _samples.clear();
    _recentIntervals.clear();
    _hasValidBPM = false;
    _peakCount = 0;
}

size_t HeartRateCalculator::getSampleCount() const { return _samples.size(); }
float HeartRateCalculator::getBPM() const { return _bpm; }
bool HeartRateCalculator::hasValidBPM() const { return _hasValidBPM; }
void HeartRateCalculator::setThreshold(uint16_t threshold) { _threshold = threshold; }
uint16_t HeartRateCalculator::getPeakCount() const { return _peakCount; }

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


std::vector<unsigned long> HeartRateCalculator::findPeaksAdaptive(uint16_t threshold) {
    std::vector<unsigned long> peaks;
    if (_samples.size() < 5) return peaks;

    // Compute dynamic refractory (default 350 ms)
    unsigned long dynamicRefractory = 350;
    if (_recentIntervals.size() >= 5) {
        std::vector<unsigned long> tmp = _recentIntervals;
        std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
        unsigned long medianInt = tmp[tmp.size() / 2];
        dynamicRefractory = std::min(600UL, std::max(250UL, (unsigned long)(0.45 * medianInt)));
    }

    for (size_t i = 2; i < _samples.size() - 2; i++) {
        uint16_t val = _samples[i].value;
        if (val > threshold &&
            val > _samples[i - 1].value &&
            val > _samples[i - 2].value &&
            val > _samples[i + 1].value &&
            val > _samples[i + 2].value) {

            if (peaks.empty() || (_samples[i].timestamp - peaks.back() > dynamicRefractory)) {
                peaks.push_back(_samples[i].timestamp);
            }
        }
    }

    Serial.print("  Dynamic refractory: ");
    Serial.print(dynamicRefractory);
    Serial.println(" ms");
    return peaks;
}

float HeartRateCalculator::calculateBPM() {
    _hasValidBPM = false;
    _peakCount = 0;

    if (_samples.size() < 100) {
        Serial.println("  Not enough samples");
        return 0.0;
    }

    // Compute stats
    uint16_t minVal = 65535, maxVal = 0;
    unsigned long sumVal = 0;
    for (auto &s : _samples) {
        if (s.value < minVal) minVal = s.value;
        if (s.value > maxVal) maxVal = s.value;
        sumVal += s.value;
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

    if (signalRange < 50) {
        Serial.println("  Signal too small - poor contact");
        return 0.0;
    }

    // Adaptive threshold: 25% above mean
    uint16_t adaptiveThreshold = avgVal + (signalRange * 0.25);
    Serial.print("  Using adaptive threshold: ");
    Serial.println(adaptiveThreshold);

    // Detect peaks
    std::vector<unsigned long> peaks = findPeaksAdaptive(adaptiveThreshold);
    _peakCount = peaks.size();
    Serial.print("  Peaks found: ");
    Serial.println(_peakCount);
    if (peaks.size() < 3) return 0.0;

    // Build intervals
    std::vector<unsigned long> intervals;
    for (size_t i = 1; i < peaks.size(); i++) {
        unsigned long interval = peaks[i] - peaks[i - 1];
        if (interval >= 250 && interval <= 2000) intervals.push_back(interval);
    }

    if (intervals.size() < 2) {
        Serial.println("  Not enough valid intervals");
        return 0.0;
    }

    // Median filter: remove extreme 10% on each side
    std::sort(intervals.begin(), intervals.end());
    if (intervals.size() > 5) {
        size_t cut = intervals.size() / 10;
        intervals = std::vector<unsigned long>(intervals.begin() + cut, intervals.end() - cut);
    }

    // Compute median interval
    std::nth_element(intervals.begin(), intervals.begin() + intervals.size() / 2, intervals.end());
    float medianInterval = intervals[intervals.size() / 2];

    // Update recent intervals buffer
    for (auto iv : intervals) {
        _recentIntervals.push_back(iv);
        if (_recentIntervals.size() > 20) _recentIntervals.erase(_recentIntervals.begin());
    }

    // Calculate BPM
    _bpm = 60000.0 / medianInterval;

    Serial.print("  Median interval: ");
    Serial.print(medianInterval);
    Serial.print(" ms  -> BPM = ");
    Serial.println(_bpm);

    if (_bpm < 30 || _bpm > 200) {
        Serial.println("  BPM out of range");
        return 0.0;
    }

    _hasValidBPM = true;
    return _bpm;
}
