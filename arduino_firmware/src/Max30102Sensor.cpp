#include "Max30102Sensor.h"
#include <Arduino.h>

// MAX30102 register addresses
#define REG_INTR_STATUS_1  0x00
#define REG_INTR_ENABLE_1  0x02
#define REG_FIFO_WR_PTR    0x04
#define REG_FIFO_RD_PTR    0x06
#define REG_FIFO_DATA      0x07
#define REG_MODE_CONFIG    0x09
#define REG_SPO2_CONFIG    0x0A
#define REG_LED1_PA        0x0C  // LED1 = IR
#define REG_LED2_PA        0x0D  // LED2 = Red

bool Max30102Sensor::begin() {
    Wire.begin();  // ensure Wire is initialized
    // Check if sensor is connected by reading a known register (e.g., part ID, not shown for brevity)

    // Reset sensor
    Wire.beginTransmission(MAX30102_ADDR);
    Wire.write(REG_MODE_CONFIG);
    Wire.write(0x40); // reset command
    Wire.endTransmission();
    delay(100); // wait for reset

    // Configure sensor: SpO2 mode, sample rate, LED pulse width, etc.
    Wire.beginTransmission(MAX30102_ADDR);
    Wire.write(REG_MODE_CONFIG);
    Wire.write(0x03); // mode = SpO2 mode (red + IR) 
    Wire.endTransmission();

    Wire.beginTransmission(MAX30102_ADDR);
    Wire.write(REG_SPO2_CONFIG);
    Wire.write(0x27); // SPO2_ADC range = 4096nA, sample rate = 100Hz, LED pulse width = 411uS
    Wire.endTransmission();

    // LED pulse amplitudes
    Wire.beginTransmission(MAX30102_ADDR);
    Wire.write(REG_LED1_PA);
    Wire.write(0x7F); // IR LED current (0x7F = max ~50mA)
    Wire.write(0x7F); // Red LED current (0x7F = max)
    Wire.endTransmission();

    // Clear FIFO pointers
    Wire.beginTransmission(MAX30102_ADDR);
    Wire.write(REG_FIFO_WR_PTR);
    Wire.write(0x00); // write pointer
    Wire.write(0x00); // overflow counter
    Wire.write(0x00); // read pointer
    Wire.endTransmission();

    // Enable FIFO rollover if desired (not done here, default should be fine)
    // Check if sensor responds properly (could read part ID, etc.)
    return true; // Assuming initialization successful
}

void Max30102Sensor::update(HealthData &data) {
    // Read a sample from the FIFO (3 bytes for IR, 3 bytes for Red per sample)
    Wire.beginTransmission(MAX30102_ADDR);
    Wire.write(REG_FIFO_DATA);
    Wire.endTransmission(false);
    Wire.requestFrom(MAX30102_ADDR, (uint8_t)6);
    if (Wire.available() < 6) {
        return; // I2C read failed
    }
    // Each sample is 18-bit value (3 bytes). Mask out the lowest 2 bits of the first byte as they are not part of 18-bit value.
    uint32_t rawIR = 0, rawRed = 0;
    rawIR |= (Wire.read() & 0x03) << 16;
    rawIR |= Wire.read() << 8;
    rawIR |= Wire.read();
    rawRed |= (Wire.read() & 0x03) << 16;
    rawRed |= Wire.read() << 8;
    rawRed |= Wire.read();

    // If no finger detected (IR reading very low), set outputs to 0
    const long FINGER_THRESHOLD = 50000; // baseline threshold for finger presence
    if (rawIR < FINGER_THRESHOLD) {
        data.heartRate = 0.0f;
        data.spo2 = 0.0f;
        // Reset tracking if finger removed
        lastBeatTime = 0;
        ratesCount = 0;
        ratesIndex = 0;
        resetCycleMeasurements();
        beatDetected = false;
        return;
    }

    // Update cycle min/max for SpO2 calculation
    if (rawIR < irMin) irMin = rawIR;
    if (rawIR > irMax) irMax = rawIR;
    if (rawRed < redMin) redMin = rawRed;
    if (rawRed > redMax) redMax = rawRed;

    // Simple beat detection: check for a significant rise in IR signal
    beatDetectedFlag = false;
    if (rawIR > FINGER_THRESHOLD && rawIR > lastIR + 1000) {  // sudden increase:contentReference[oaicite:12]{index=12}
        // Potential beat detected
        uint32_t currentTime = millis();
        uint32_t interval = currentTime - lastBeatTime;
        if (lastBeatTime != 0 && interval > 0) {
            // Calculate BPM from interval
            float bpm = 60000.0 / interval;
            // Only consider valid heart rate in a plausible range
            if (bpm >= 20 && bpm <= 220) {
                // Store BPM in rolling array for averaging
                rates[ratesIndex] = (uint8_t) bpm;
                if (ratesCount < RATE_SIZE) {
                    ratesCount++;
                }
                ratesIndex = (ratesIndex + 1) % RATE_SIZE;
                // Compute average BPM over last N beats
                uint16_t total = 0;
                for (uint8_t i = 0; i < ratesCount; ++i) {
                    total += rates[i];
                }
                float avgBPM = (float)total / ratesCount;
                data.heartRate = avgBPM;
            }
            // Calculate SpO2 at the end of a pulse cycle (we have min/max of previous cycle)
            if (!firstCycle) {
                if (irMax > irMin && redMax > redMin) {
                    float dcIR = (float) irMin;
                    float dcRed = (float) redMin;
                    float acIR = (float) (irMax - irMin);
                    float acRed = (float) (redMax - redMin);
                    // Ratio of ratios R:contentReference[oaicite:13]{index=13}
                    float R = (acRed / dcRed) / (acIR / dcIR);
                    float computedSpO2 = 110.0f - 25.0f * R;  // empirical formula:contentReference[oaicite:14]{index=14}
                    if (computedSpO2 > 100.0f) computedSpO2 = 100.0f;
                    if (computedSpO2 < 0.0f)   computedSpO2 = 0.0f;
                    data.spo2 = computedSpO2;
                }
            } else {
                // Skip SpO2 calculation for the first detected cycle (for calibration)
                data.spo2 = 0.0f;
                firstCycle = false;
            }
            // Reset for next cycle
            resetCycleMeasurements();
        }
        lastBeatTime = currentTime;
        beatDetectedFlag = true;
        lastBeatInterval = interval;
    }

    // If no new beat, output the last known heart rate (it will remain until updated)
    // (Optionally, one could slowly decrement or adjust heartRate if no beat detected for a while)

    // Save current IR value for next beat detection comparison
    lastIR = rawIR;

    // Set the beatDetected flag accessible to controller
    beatDetected = beatDetectedFlag;
}
