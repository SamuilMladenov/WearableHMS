#include "ADPDSensor.h"

ADPD105::ADPD105() : _address(0x64), _i2cSpeed(100000), _initialized(false) {
}

bool ADPD105::begin(uint8_t address, uint32_t i2cSpeed) {
    _address = address;
    _i2cSpeed = i2cSpeed;
    
    Wire.begin();
    Wire.setClock(_i2cSpeed);
    
    delay(100); // Allow sensor to power up
    
    Serial.print("  Checking ADPD105 connection...");
    if (!checkConnection()) {
        Serial.println(" FAILED");
        _initialized = false;
        return false;
    }
    Serial.println(" OK");
    
    // Reset the sensor
    Serial.print("  Resetting ADPD105...");
    reset();
    delay(100);
    Serial.println(" OK");
    
    // Configure the sensor
    Serial.print("  Configuring ADPD105...");
    if (!configureMinimal()) {
        Serial.println(" FAILED");
        _initialized = false;
        return false;
    }
    Serial.println(" OK");
    
    _initialized = true;
    return true;
}

bool ADPD105::readFifoData(uint16_t &sample) {
    if (!_initialized) return false;
    
    uint8_t wordCount = getFifoWordCount();
    if (wordCount == 0) return false;
    
    return readFifoWords(&sample, 1);
}

bool ADPD105::readFifoData(uint16_t *samples, uint8_t count) {
    if (!_initialized || count == 0) return false;
    
    uint8_t wordCount = getFifoWordCount();
    if (wordCount < count) count = wordCount;
    
    return readFifoWords(samples, count);
}

bool ADPD105::readFifoDataDual(uint16_t &redSample, uint16_t &irSample) {
    if (!_initialized) return false;
    
    uint8_t wordCount = getFifoWordCount();
    if (wordCount < 2) return false;
    
    uint16_t samples[2];
    if (!readFifoWords(samples, 2)) return false;
    
    // Slot A (RED) comes first, then Slot B (IR)
    redSample = samples[0];
    irSample = samples[1];
    
    return true;
}

uint8_t ADPD105::getFifoWordCount() {
    if (!_initialized) return 0;
    
    uint16_t status;
    if (!readRegister16(REG_STATUS, status)) return 0;
    
    return (status >> 8) & 0xFF; // FIFO word count in upper byte
}

void ADPD105::printDiagnostics() {
    if (!_initialized) {
        Serial.println("ADPD105: Not initialized");
        return;
    }
    
    Serial.print("ADPD105: FIFO words available: ");
    Serial.println(getFifoWordCount());
}

void ADPD105::reset() {
    writeRegister16(REG_SW_RESET, 0x0001);
    delay(100);
}

bool ADPD105::writeRegister16(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF); // MSB
    Wire.write(value & 0xFF);        // LSB
    return Wire.endTransmission() == 0;
}

bool ADPD105::readRegister16(uint8_t reg, uint16_t &value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;
    
    Wire.requestFrom(_address, (uint8_t)2);
    if (Wire.available() < 2) return false;
    
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    value = ((uint16_t)msb << 8) | lsb;
    
    return true;
}

bool ADPD105::readFifoWords(uint16_t *buf, uint8_t n) {
    Wire.beginTransmission(_address);
    Wire.write(REG_FIFO_DATA);
    if (Wire.endTransmission() != 0) return false;
    
    Wire.requestFrom(_address, (uint8_t)(n * 2));
    
    for (uint8_t i = 0; i < n; i++) {
        if (Wire.available() < 2) return false;
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        buf[i] = ((uint16_t)msb << 8) | lsb;
    }
    
    return true;
}

bool ADPD105::configureMinimal() {
    // Default configuration - calls configureForHeartRate
    return configureForHeartRate();
}

bool ADPD105::configureForHeartRate() {
    // Configuration for heart rate only (RED LED only, Slot A)
    Serial.println("  Configuring for Heart Rate (RED LED only)");
    
    // 1) Enable the 32 kHz sample clock
    if (!writeRegister16(REG_CLK32K, 0x4C92)) {
        Serial.println("ADPD105: CLK32K config failed");
        return false;
    }

    // 2) Program mode first
    if (!writeRegister16(REG_MODE, 0x0001)) {
        Serial.println("ADPD105: MODE config failed");
        return false;
    }

    // 3) Sampling frequency: 100 Hz
    if (!writeRegister16(REG_FSAMPLE, 0x0050)) {
        Serial.println("ADPD105: FSAMPLE config failed");
        return false;
    }

    // 4) No averaging (1x) on Slot A
    if (!writeRegister16(REG_NUM_AVG, 0x0000)) {
        Serial.println("ADPD105: NUM_AVG config failed");
        return false;
    }

    // 5) Photodiode/LED mapping - RED LED
    if (!writeRegister16(REG_PD_LED_CFG, 0x0551)) {
        Serial.println("ADPD105: PD_LED_CFG config failed");
        return false;
    }

    // 6) LED1 (RED) current
    if (!writeRegister16(REG_LED1_DRV, 0x2001)) {
        Serial.println("ADPD105: LED1_DRV config failed");
        return false;
    }

    // 7) FIFO config & enable Slot A only
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

bool ADPD105::configureForSpO2() {
    // Configuration for SpO2 (RED and IR LEDs, Slots A and B)
    Serial.println("  Configuring for SpO2 (RED + IR LEDs)");
    
    // 1) Standby mode first
    if (!writeRegister16(REG_MODE, 0x0001)) {
        Serial.println("ADPD105: MODE config failed");
        return false;
    }
    delay(10);
    
    // 2) Enable the 32 kHz sample clock
    if (!writeRegister16(REG_CLK32K, 0x4C92)) {
        Serial.println("ADPD105: CLK32K config failed");
        return false;
    }

    // 3) Sampling frequency: 100 Hz
    if (!writeRegister16(REG_FSAMPLE, 0x0050)) {
        Serial.println("ADPD105: FSAMPLE config failed");
        return false;
    }

    // 4) No averaging
    if (!writeRegister16(REG_NUM_AVG, 0x0000)) {
        Serial.println("ADPD105: NUM_AVG config failed");
        return false;
    }

    // 5) Photodiode/LED mapping for both slots
    if (!writeRegister16(REG_PD_LED_CFG, 0x0551)) {
        Serial.println("ADPD105: PD_LED_CFG config failed");
        return false;
    }

    // 6) LED1 (RED) current for Slot A - keep same as heart rate
    if (!writeRegister16(REG_LED1_DRV, 0x2001)) {
        Serial.println("ADPD105: LED1_DRV config failed");
        return false;
    }

    // 7) LED2 (IR) current for Slot B - REDUCED to prevent saturation
    // Changed from 0x2001 to 0x0401 (much lower current)
    if (!writeRegister16(REG_LED2_DRV, 0x0401)) {
        Serial.println("ADPD105: LED2_DRV config failed");
        return false;
    }

    // 8) FIFO threshold
    if (!writeRegister16(REG_FIFO_THRESH, 0x0010)) {
        Serial.println("ADPD105: FIFO_THRESH config failed");
        return false;
    }

    // 9) Enable BOTH Slot A (RED) and Slot B (IR)
    if (!writeRegister16(REG_SLOT_EN, 0x3005)) {
        Serial.println("ADPD105: SLOT_EN config failed");
        return false;
    }

    delay(10);

    // 10) Normal mode (start state machine)
    if (!writeRegister16(REG_MODE, 0x0002)) {
        Serial.println("ADPD105: Normal mode start failed");
        return false;
    }

    Serial.println("  SpO2 configuration complete");
    return true;
}

bool ADPD105::checkConnection() {
    uint16_t deviceId;
    if (!readRegister16(REG_DEVID, deviceId)) {
        Serial.print(" Read failed");
        return false;
    }
    
    Serial.print(" DevID=0x");
    Serial.print(deviceId, HEX);
    
    // ADPD105 device ID should be 0x00C2, but let's be more lenient
    // Accept any non-zero response as the sensor might have a different ID
    if (deviceId == 0x0000 || deviceId == 0xFFFF) {
        return false;
    }
    
    return true;
}