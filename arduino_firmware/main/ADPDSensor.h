#pragma once

#include <Wire.h>
#include <Arduino.h>

class ADPD105 {
public:
    ADPD105();
    bool begin(uint8_t address = 0x64, uint32_t i2cSpeed = 100000, bool forSpO2 = false);
    bool readFifoData(uint16_t &sample);
    bool readFifoData(uint16_t *samples, uint8_t count);
    bool readFifoDataDual(uint16_t &redSample, uint16_t &irSample);
    uint8_t getFifoWordCount();
    void printDiagnostics();
    void reset();
    bool configureForHeartRate();
    bool configureForSpO2();
    
private:
    uint8_t _address;
    uint32_t _i2cSpeed;
    bool _initialized;
    
    // Register addresses
    static const uint8_t REG_STATUS = 0x00;
    static const uint8_t REG_INT_MASK = 0x01;
    static const uint8_t REG_FIFO_THRESH = 0x06;
    static const uint8_t REG_DEVID = 0x08;
    static const uint8_t REG_SW_RESET = 0x0F;
    static const uint8_t REG_MODE = 0x10;
    static const uint8_t REG_SLOT_EN = 0x11;
    static const uint8_t REG_FSAMPLE = 0x12;
    static const uint8_t REG_NUM_AVG = 0x15;
    static const uint8_t REG_PD_LED_CFG = 0x14;
    static const uint8_t REG_LED1_DRV = 0x23;
    static const uint8_t REG_LED2_DRV = 0x24;
    static const uint8_t REG_CLK32K = 0x4B;
    static const uint8_t REG_FIFO_DATA = 0x60;
    
    bool writeRegister16(uint8_t reg, uint16_t value);
    bool readRegister16(uint8_t reg, uint16_t &value);
    bool readFifoWords(uint16_t *buf, uint8_t n);
    bool configureMinimal();
    bool checkConnection();
};