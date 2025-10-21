#include "SpO2Calculator.h"
#include <math.h>

SpO2Calculator::SpO2Calculator() 
    : _spo2(0), 
      _hasValidSpO2(false),
      _ratioOfRatios(0) {
    _samples.reserve(2000); // Reserve space for ~20 seconds at 100Hz
}

void SpO2Calculator::addSample(unsigned long timestamp, uint16_t redValue, uint16_t irValue) {
    _samples.push_back({timestamp, redValue, irValue});
}

void SpO2Calculator::clearSamples() {
    _samples.clear();
    _hasValidSpO2 = false;
    _ratioOfRatios = 0;
}

size_t SpO2Calculator::getSampleCount() const {
    return _samples.size();
}

float SpO2Calculator::getSpO2() const {
    return _spo2;
}

bool SpO2Calculator::hasValidSpO2() const {
    return _hasValidSpO2;
}

void SpO2Calculator::printSampleStats() const {
    if (_samples.empty()) {
        Serial.println("  No samples collected");
        return;
    }
    
    Serial.println("  Sample values (first 10 pairs):");
    for (size_t i = 0; i < 10 && i < _samples.size(); i++) {
        Serial.print("    [");
        Serial.print(i);
        Serial.print("] RED=");
        Serial.print(_samples[i].redValue);
        Serial.print(" IR=");
        Serial.println(_samples[i].irValue);
    }
}

void SpO2Calculator::calculateACDC(const std::vector<uint16_t>& signal, float& ac, float& dc) {
    if (signal.empty()) {
        ac = 0;
        dc = 0;
        return;
    }
    
    // Calculate DC (average)
    unsigned long sum = 0;
    for (uint16_t val : signal) {
        sum += val;
    }
    dc = (float)sum / signal.size();
    
    // Calculate AC (standard deviation as proxy for pulsatile component)
    float variance = 0;
    for (uint16_t val : signal) {
        float diff = val - dc;
        variance += diff * diff;
    }
    variance /= signal.size();
    ac = sqrt(variance);
}

float SpO2Calculator::applyCalibration(float ratio) {
    // Empirical calibration formula (typical for pulse oximeters)
    // SpO2 = 110 - 25 * R
    // This is a simplified linear approximation
    // Real devices use more complex calibration curves
    
    float spo2 = 110.0 - 25.0 * ratio;
    
    // Clamp to realistic range
    if (spo2 > 100.0) spo2 = 100.0;
    if (spo2 < 70.0) spo2 = 70.0;
    
    return spo2;
}

float SpO2Calculator::calculateSpO2() {
    _hasValidSpO2 = false;
    
    if (_samples.size() < 100) {
        Serial.println("  Not enough samples for SpO2");
        return 0.0;
    }
    
    // Check for saturation first
    int saturatedRed = 0, saturatedIR = 0;
    for (const auto& sample : _samples) {
        if (sample.redValue >= 65535) saturatedRed++;
        if (sample.irValue >= 65535) saturatedIR++;
    }
    
    float redSatPercent = (saturatedRed * 100.0) / _samples.size();
    float irSatPercent = (saturatedIR * 100.0) / _samples.size();
    
    Serial.print("  Saturation check - RED: ");
    Serial.print(redSatPercent, 1);
    Serial.print("%, IR: ");
    Serial.print(irSatPercent, 1);
    Serial.println("%");
    
    if (redSatPercent > 10.0) {
        Serial.println("  RED signal saturated - reduce LED1 current");
        return 0.0;
    }
    
    if (irSatPercent > 10.0) {
        Serial.println("  IR signal saturated - reduce LED2 current");
        return 0.0;
    }
    
    // Separate RED and IR signals
    std::vector<uint16_t> redSignal;
    std::vector<uint16_t> irSignal;
    
    redSignal.reserve(_samples.size());
    irSignal.reserve(_samples.size());
    
    for (const auto& sample : _samples) {
        redSignal.push_back(sample.redValue);
        irSignal.push_back(sample.irValue);
    }
    
    // Calculate AC and DC components for both wavelengths
    float acRed, dcRed, acIR, dcIR;
    
    calculateACDC(redSignal, acRed, dcRed);
    calculateACDC(irSignal, acIR, dcIR);
    
    Serial.print("  RED - AC: ");
    Serial.print(acRed);
    Serial.print(" DC: ");
    Serial.println(dcRed);
    
    Serial.print("  IR  - AC: ");
    Serial.print(acIR);
    Serial.print(" DC: ");
    Serial.println(dcIR);
    
    // Check for valid signals
    if (dcRed < 100 || dcIR < 100) {
        Serial.println("  DC components too low - poor contact");
        return 0.0;
    }
    
    if (acRed < 1 || acIR < 1) {
        Serial.println("  AC components too low - no pulsatile signal");
        return 0.0;
    }
    
    // Calculate ratio of ratios: R = (AC_red/DC_red) / (AC_ir/DC_ir)
    float redRatio = acRed / dcRed;
    float irRatio = acIR / dcIR;
    
    if (irRatio < 0.0001) {
        Serial.println("  IR ratio too small");
        return 0.0;
    }
    
    _ratioOfRatios = redRatio / irRatio;
    
    Serial.print("  Ratio of Ratios (R): ");
    Serial.println(_ratioOfRatios, 4);
    
    // Sanity check on ratio
    if (_ratioOfRatios < 0.2 || _ratioOfRatios > 2.0) {
        Serial.print("  Ratio out of expected range (0.2-2.0)");
        Serial.println();
        return 0.0;
    }
    
    // Apply calibration
    _spo2 = applyCalibration(_ratioOfRatios);
    
    Serial.print("  Calculated SpO2: ");
    Serial.print(_spo2);
    Serial.println("%");
    
    _hasValidSpO2 = true;
    return _spo2;
}