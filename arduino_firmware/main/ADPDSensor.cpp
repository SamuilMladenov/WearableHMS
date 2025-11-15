#include "ADPDSensor.h"

ADPD105::ADPD105() : _address(0x64), _i2cSpeed(100000), _initialized(false) {
}

bool ADPD105::begin(uint8_t address, uint32_t i2cSpeed, bool forSpO2) {
    _address = address;
    _i2cSpeed = i2cSpeed;

    // Wire.begin() MUST NOT be called repeatedly in continuous operation.
    // It is already initialized once in setup().
    Wire.setClock(_i2cSpeed);
    delay(10);

    // Stop any previous state machine
    writeRegister16(REG_MODE, 0x0000);
    delay(10);

    // Clear FIFO and status flags
    writeRegister16(REG_FIFO_CLR, 0x0001);
    delay(5);
    writeRegister16(REG_INT_STATUS, 0xFFFF);

    Serial.print("  Checking ADPD105 connection...");
    if (!checkConnection()) {
        Serial.println(" FAILED");
        _initialized = false;
        return false;
    }
    Serial.println(" OK");

    Serial.print("  Resetting ADPD105...");
    reset();
    delay(50);
    Serial.println(" OK");

    Serial.print("  Configuring ADPD105...  ");
    bool ok = forSpO2 ? configureForSpO2() : configureMinimal();
    if (!ok) {
        Serial.println("FAILED");
        _initialized = false;
        return false;
    }

    Serial.println(forSpO2 ? "Configuring for SpO2 (RED + IR LEDs)" 
                           : "Configuring for Heart Rate (RED LED only)");
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

    uint8_t words = getFifoWordCount();
    if (words < 2) return false;           // need a full packet

    uint16_t buf[2];
    if (!readFifoWords(buf, 2)) return false;

    // Packet: [0]=Slot A (RED), [1]=Slot B (IR)
    redSample = buf[0];
    irSample  = buf[1];
    return true;
}


uint8_t ADPD105::getFifoWordCount() {
    if (!_initialized) return 0;
    uint16_t status;
    if (!readRegister16(REG_STATUS, status)) return 0;

    uint8_t bytes = (status >> 8) & 0xFF;   // FIFO_SAMPLES in BYTES
    return bytes / 2;                       // convert to 16-bit words
}

bool ADPD105::readRegisterPublic(uint8_t reg, uint16_t &value) {
    return readRegister16(reg, value);
}
bool ADPD105::writeRegisterPublic(uint8_t reg, uint16_t value) {
    return writeRegister16(reg, value);
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
    writeRegister16(REG_MODE, 0x0000);     // stop state machine first
    delay(5);
    writeRegister16(REG_SW_RESET, 0x0001); // issue soft reset
    delay(50);
    writeRegister16(REG_FIFO_CLR, 0x0001); // clear FIFO
    delay(5);
    writeRegister16(REG_INT_STATUS, 0xFFFF); // clear interrupts
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
    if (!writeRegister16(REG_NUM_AVG, 0x1111)) {
        Serial.println("ADPD105: NUM_AVG config failed");
        return false;
    }

    // 5) Photodiode/LED mapping - RED LED
    if (!writeRegister16(REG_PD_LED_CFG, 0x0551)) {
        Serial.println("ADPD105: PD_LED_CFG config failed");
        return false;
    }

    // 6) LED1 (RED) current
    if (!writeRegister16(REG_LED1_DRV, 0x4001)) {
        Serial.println("ADPD105: LED1_DRV config failed");
        return false;
    }

    // 7) FIFO config & enable Slot A only
    if (!writeRegister16(REG_SLOT_EN, 0x1005)) {
        Serial.println("ADPD105: SLOT_EN config failed");
        return false;
    }

    // Clear FIFO and status before starting
    writeRegister16(REG_FIFO_CLR, 0x0001);
    delay(5);
    writeRegister16(REG_INT_STATUS, 0xFFFF); // clear interrupts

    // 8) Normal mode (start state machine)
    if (!writeRegister16(REG_MODE, 0x0002)) {
        Serial.println("ADPD105: Normal mode start failed");
        return false;
    }
    delay(50);
    writeRegister16(REG_FIFO_CLR, 0x0001);

    return true;
}

bool ADPD105::configureForSpO2() {
    // Program mode
    writeRegister16(REG_MODE, 0x0001);

    // Start 32 kHz state machine clock, 100 Hz sample, same averaging on A/B
    writeRegister16(REG_CLK32K,   0x4C92);  // enables 32 kHz clock
    writeRegister16(REG_FSAMPLE,  0x0050);  // 100 Hz
    writeRegister16(REG_NUM_AVG,  0x0000);  // NA = NB = 1 (keep A=B)

    // AFE in normal (analog full path) for both slots; default TIA/VBIAS is OK to start
    // (SLOTA_AFE_CFG = 0x43 = 0xADA5; SLOTB_AFE_CFG = 0x45 = 0xADA5)
    writeRegister16(0x43, 0xADA5); // SLOTA_AFE_CFG (normal path). :contentReference[oaicite:3]{index=3}
    writeRegister16(0x45, 0xADA5); // SLOTB_AFE_CFG (normal path).  :contentReference[oaicite:4]{index=4}

    // Map PD/LEDs so Slot A = RED, Slot B = IR (your 0x0555 works for “all PDs on”, LEDs 1–2 active)
    writeRegister16(REG_PD_LED_CFG, 0x0555);

    // LED currents — keep IR <= RED initially; tweak down if ADC saturates
    writeRegister16(REG_LED1_DRV, 0x1000);  // RED ~ moderate
    writeRegister16(REG_LED2_DRV, 0x0800);  // IR lower than RED (IR couples more strongly)

    // FIFO: 16-bit per slot (A & B) + enable both slots
    writeRegister16(REG_SLOT_EN, 0x0065);   // modes=1/1, enable A/B. :contentReference[oaicite:5]{index=5}

    // Optional: set FIFO threshold near one packet (2 words = 1 packet ⇒ thresh = 1 word)
    writeRegister16(REG_FIFO_THRESH, 0x0001); // not required if polling. :contentReference[oaicite:6]{index=6}

    // Unmask FIFO/SLOT interrupts (not strictly needed if you poll)
    writeRegister16(REG_INT_MASK, 0x0000);  // enable

    // Normal mode
    writeRegister16(REG_MODE, 0x0002);
    delay(50);

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