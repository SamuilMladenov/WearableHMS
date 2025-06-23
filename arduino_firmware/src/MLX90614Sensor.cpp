#include "MLX90614Sensor.h"

bool MLX90614Sensor::begin() {
    // Wire should already be initialized in main. Just verify sensor communication.
    Wire.beginTransmission(MLX90614_I2C_ADDR);
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        // Sensor did not ACK - not found
        return false;
    }
    return true;
}

void MLX90614Sensor::update(HealthData &data) {
    // MLX90614 object temperature register is 0x07 (RAM)
    const uint8_t OBJ_TEMP_REGISTER = 0x07;
    Wire.beginTransmission(MLX90614_I2C_ADDR);
    Wire.write(OBJ_TEMP_REGISTER);
    if (Wire.endTransmission(false) != 0) {
        return; // error
    }
    // Request 3 bytes (2-byte temperature + 1-byte PEC)
    Wire.requestFrom((int)MLX90614_I2C_ADDR, 3);
    if (Wire.available() >= 3) {
        uint8_t lsb = Wire.read();
        uint8_t msb = Wire.read();
        uint8_t pec = Wire.read(); // PEC (packet error checking byte, not used here)
        uint16_t raw = ((uint16_t)msb << 8) | lsb;
        // According to datasheet, temperature in K = raw * 0.02
        // So temp in Celsius = raw * 0.02 - 273.15
        float tempC = raw * 0.02f - 273.15f;
        data.bodyTemp = tempC;
    }
}
