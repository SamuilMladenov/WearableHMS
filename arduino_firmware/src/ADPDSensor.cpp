#include "ADPD105Sensor.h"
#include <math.h>

ADPD105Sensor::ADPD105Sensor()
    : dcRed(0), dcIR(0), envHR(0), bpmSmooth(0),
      lastAC(0), lastPeakMs(0), lastBPM(NAN) {}

void ADPD105Sensor::begin() {
    Wire.begin();
    Wire.setClock(100000);
    writeRegister(REG_SW_RESET, 0x0001);
    delay(5);
    if (!configSensor()) {
        Serial.println("ADPD105 config failed!");
    } else {
        Serial.println("ADPD105 ready.");
    }
}

bool ADPD105Sensor::configSensor() {
    return
        writeRegister(REG_CLK32K, 0x4C92) &&
        writeRegister(REG_MODE, 0x0001) &&
        writeRegister(REG_FSAMPLE, 0x0050) &&
        writeRegister(REG_NUM_AVG, 0x0000) &&
        writeRegister(REG_PD_LED_SEL, 0x0559) &&
        writeRegister(REG_LED1_DRV, 0x2018) &&
        writeRegister(REG_LED2_DRV, 0x2018) &&
        writeRegister(REG_SLOT_EN, 0x1065) &&
        writeRegister(REG_MODE, 0x0002);
}

bool ADPD105Sensor::writeRegister(uint8_t reg, uint16_t val) {
    Wire.beginTransmission(ADPD105_ADDR);
    Wire.write(reg);
    Wire.write(uint8_t(val >> 8));
    Wire.write(uint8_t(val & 0xFF));
    return Wire.endTransmission() == 0;
}

bool ADPD105Sensor::readRegister(uint8_t reg, uint16_t &out) {
    Wire.beginTransmission(ADPD105_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(ADPD105_ADDR, (uint8_t)2) != 2) return false;
    out = (uint16_t(Wire.read()) << 8) | Wire.read();
    return true;
}

bool ADPD105Sensor::readFifoWords(uint8_t n, uint16_t *buf) {
    Wire.beginTransmission(ADPD105_ADDR);
    Wire.write(REG_FIFO_DATA);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(ADPD105_ADDR, (uint8_t)(2 * n)) != 2 * n) return false;
    for (uint8_t i = 0; i < n; i++) {
        buf[i] = (uint16_t(Wire.read()) << 8) | Wire.read();
    }
    return true;
}

void ADPD105Sensor::processSample(uint16_t redRaw, uint16_t irRaw) {
    // EWMA filters
    float alphaDC = 0.995f;
    float alphaEnv = 0.90f;
    float alphaBPM = 0.90f;

    float red = redRaw, ir = irRaw;
    dcRed = alphaDC * dcRed + (1 - alphaDC) * red;
    dcIR = alphaDC * dcIR + (1 - alphaDC) * ir;
    float combinedAC = ((red - dcRed) + (ir - dcIR)) / 2.0f;

    float envNow = envHR = alphaEnv * envHR + (1 - alphaEnv) * fabsf(combinedAC);
    float thr = THRESH_FRAC * envNow;

    bool upCross = (lastAC < thr) && (combinedAC >= thr);
    lastAC = combinedAC;

    if (upCross) {
        uint32_t now = millis();
        if (lastPeakMs != 0) {
            uint32_t ibi = now - lastPeakMs;
            if (ibi >= IBI_MIN_MS && ibi <= IBI_MAX_MS) {
                float bpm = 60000.0f / ibi;
                lastBPM = bpmSmooth = alphaBPM * bpmSmooth + (1 - alphaBPM) * bpm;
            }
        }
        lastPeakMs = now;
    }
}

void ADPD105Sensor::update() {
    uint16_t status = 0;
    if (!readRegister(REG_STATUS, status)) return;
    uint8_t bytesAvail = uint8_t(status >> 8);

    while (bytesAvail >= 4) { // 2 words = RED + IR
        uint16_t ab[2];
        if (!readFifoWords(2, ab)) break;
        processSample(ab[0], ab[1]);
        if (!readRegister(REG_STATUS, status)) break;
        bytesAvail = uint8_t(status >> 8);
    }
}

float ADPD105Sensor::getHeartRate() const {
    return lastBPM;
}
