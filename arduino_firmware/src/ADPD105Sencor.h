#pragma once
#include <Arduino.h>
#include <Wire.h>

#define ADPD105_ADDR 0x64

class ADPD105Sensor {
public:
    ADPD105Sensor();
    void begin();
    void update();
    float getHeartRate() const;

private:
    // Registers
    static const uint8_t REG_STATUS = 0x00;
    static const uint8_t REG_SW_RESET = 0x0F;
    static const uint8_t REG_MODE = 0x10;
    static const uint8_t REG_SLOT_EN = 0x11;
    static const uint8_t REG_FSAMPLE = 0x12;
    static const uint8_t REG_PD_LED_SEL = 0x14;
    static const uint8_t REG_NUM_AVG = 0x15;
    static const uint8_t REG_LED1_DRV = 0x23;
    static const uint8_t REG_LED2_DRV = 0x24;
    static const uint8_t REG_CLK32K = 0x4B;
    static const uint8_t REG_FIFO_DATA = 0x60;

    // Filter and detection
    float dcRed, dcIR, envHR, bpmSmooth;
    float lastAC;
    uint32_t lastPeakMs;
    float lastBPM;

    // Config
    const float THRESH_FRAC = 0.35f;
    const uint16_t IBI_MIN_MS = 300;
    const uint16_t IBI_MAX_MS = 2000;

    // Internal helpers
    bool writeRegister(uint8_t reg, uint16_t val);
    bool readRegister(uint8_t reg, uint16_t &out);
    bool readFifoWords(uint8_t n, uint16_t *buf);
    bool configSensor();
    void processSample(uint16_t redRaw, uint16_t irRaw);
};
