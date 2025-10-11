#include "ADPDSensor.h"

ADPD105::ADPD105() : _address(0x64), _i2cSpeed(100000), _initialized(false) {}

bool ADPD105::begin(uint8_t address, uint32_t i2cSpeed) {
    _address = address;
    _i2cSpeed = i2cSpeed;
    
    Wire.begin();
    Wire.setClock(_i2cSpeed);
    
    // Check if sensor is connected
    if (!checkConnection()) {
        Serial.println("ADPD105: Not found at I2C address");
        return false;
    }
    
    // Perform software reset
    reset();
    delay(5);
    
    // Configure the sensor
    if (!configureMinimal()) {
        Serial.println("ADPD105: Configuration failed");
        return false;
    }
    
    _initialized = true;
    Serial.println("ADPD105: Initialized successfully");
    return true;
}

bool ADPD105::checkConnection() {
    Wire.beginTransmission(_address);
    return (Wire.endTransmission() == 0);
}

void ADPD105::reset() {
    writeRegister16(REG_SW_RESET, 0x0001);
}

bool ADPD105::configureMinimal() {
    // 1) Enable the 32 kHz sample clock
    if (!writeRegister16(REG_CLK32K, 0x4C92)) {
        Serial.println("ADPD105: CLK32K config failed");
        return false;
    }

    // 2) Program mode
    if (!writeRegister16(REG_MODE, 0x0001)) {
        Serial.println("ADPD105: MODE config failed");
        return false;
    }

    // 3) Sampling frequency: 100 Hz → FSAMPLE_REG = 0x0050
    if (!writeRegister16(REG_FSAMPLE, 0x0050)) {
        Serial.println("ADPD105: FSAMPLE config failed");
        return false;
    }

    // 4) No averaging (1x) on Slot A
    if (!writeRegister16(REG_NUM_AVG, 0x0000)) {
        Serial.println("ADPD105: NUM_AVG config failed");
        return false;
    }

    // 5) Photodiode/LED mapping
    if (!writeRegister16(REG_PD_LED_CFG, 0x0551)) {
        Serial.println("ADPD105: PD_LED_CFG config failed");
        return false;
    }

    // 6) LED1 modest current
    if (!writeRegister16(REG_LED1_DRV, 0x2001)) {
        Serial.println("ADPD105: LED1_DRV config failed");
        return false;
    }

    // 7) FIFO config & enable Slot A
    if (!writeRegister16(REG_SLOT_EN, 0x1005)) {
        Serial.println("ADPD105: SLOT_EN config failed");
        return false;
    }

    // 8) Normal mode (start state machine)
    if (!writeRegister16(REG_MODE, 0x0002)) {
        Serial.println("ADPD105: Normal mode start failed");
        return false;
    }

    return true;
}

bool ADPD105::writeRegister16(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(uint8_t(value >> 8));
    Wire.write(uint8_t(value & 0xFF));
    return (Wire.endTransmission() == 0);
}

bool ADPD105::readRegister16(uint8_t reg, uint16_t &value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    
    if (Wire.requestFrom(_address, (uint8_t)2) != 2) return false;
    
    uint16_t msb = Wire.read();
    uint16_t lsb = Wire.read();
    value = (msb << 8) | lsb;
    return true;
}

bool ADPD105::readFifoWords(uint16_t *buf, uint8_t n) {
    Wire.beginTransmission(_address);
    Wire.write(REG_FIFO_DATA);
    if (Wire.endTransmission(false) != 0) return false;
    
    if (Wire.requestFrom(_address, uint8_t(n * 2)) != n * 2) return false;
    
    for (uint8_t i = 0; i < n; i++) {
        uint16_t msb = Wire.read();
        uint16_t lsb = Wire.read();
        buf[i] = (msb << 8) | lsb;
    }
    return true;
}

uint8_t ADPD105::getFifoWordCount() {
    uint16_t status = 0;
    if (!readRegister16(REG_STATUS, status)) {
        return 0;
    }
    return uint8_t(status >> 8); // FIFO_SAMPLES (words)
}

bool ADPD105::readFifoData(uint16_t &sample) {
    return readFifoData(&sample, 1);
}

bool ADPD105::readFifoData(uint16_t *samples, uint8_t count) {
    if (!_initialized) return false;
    
    uint8_t wordsAvailable = getFifoWordCount();
    if (wordsAvailable < count) {
        return false; // Not enough data available
    }
    
    return readFifoWords(samples, count);
}

void ADPD105::printDiagnostics() {
    Serial.println("ADPD105 Diagnostics:");
    Serial.print("  Connected: "); Serial.println(checkConnection() ? "Yes" : "No");
    Serial.print("  Initialized: "); Serial.println(_initialized ? "Yes" : "No");
    Serial.print("  I2C Address: 0x"); Serial.println(_address, HEX);
    Serial.print("  I2C Speed: "); Serial.print(_i2cSpeed); Serial.println(" Hz");
    
    if (_initialized) {
        uint16_t devId = 0;
        if (readRegister16(REG_DEVID, devId)) {
            Serial.print("  Device ID: 0x"); Serial.println(devId, HEX);
        }
        
        uint8_t fifoCount = getFifoWordCount();
        Serial.print("  FIFO Words Available: "); Serial.println(fifoCount);
        
        uint16_t status = 0;
        if (readRegister16(REG_STATUS, status)) {
            Serial.print("  Status: 0x"); Serial.println(status, HEX);
        }
    }
}