#include "MLX90614Sensor.h"

MLX90614::MLX90614() : _address(0x5A), _initialized(false) {
}

bool MLX90614::begin(uint8_t address) {
    _address = address;
    
    Wire.begin();
    Wire.setClock(100000); // 100kHz for MLX90614
    
    delay(100);
    
    Serial.print("  Testing MLX90614...");
    // Try to read temperature to verify sensor
    float temp = readTemperature();
    
    if (temp < -50 || temp > 150) { // Unrealistic temperature range
        Serial.print(" FAILED (temp=");
        Serial.print(temp);
        Serial.println(")");
        _initialized = false;
        return false;
    }
    
    Serial.print(" OK (temp=");
    Serial.print(temp);
    Serial.println("°C)");
    _initialized = true;
    
    return _initialized;
}

float MLX90614::readTemperature() {
    uint16_t rawTemp;
    if (!readRegister16(0x07, rawTemp)) { // 0x07 = Object temperature register
        return -999.0;
    }
    
    // Check for error values
    if (rawTemp == 0x0000 || rawTemp == 0xFFFF) {
        return -999.0;
    }
    
    // Convert to Celsius
    float temp = rawTemp * 0.02 - 273.15;
    
    return temp;
}

float MLX90614::readAmbientTemperature() {
    if (!_initialized) return -1.0;
    
    uint16_t rawTemp;
    if (!readRegister16(0x06, rawTemp)) { // 0x06 = Ambient temperature register
        return -1.0;
    }
    
    float temp = rawTemp * 0.02 - 273.15;
    
    return temp;
}

void MLX90614::printDiagnostics() {
    if (!_initialized) {
        Serial.println("MLX90614: Not initialized");
        return;
    }
    
    float objTemp = readTemperature();
    float ambTemp = readAmbientTemperature();
    
    Serial.print("MLX90614: Object=");
    Serial.print(objTemp);
    Serial.print("°C, Ambient=");
    Serial.print(ambTemp);
    Serial.println("°C");
}

bool MLX90614::readRegister16(uint8_t reg, uint16_t &value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    
    Wire.requestFrom(_address, (uint8_t)3); // 2 data bytes + 1 PEC
    if (Wire.available() < 3) return false;
    
    uint8_t lsb = Wire.read();
    uint8_t msb = Wire.read();
    Wire.read(); // PEC byte (not checking for simplicity)
    
    value = ((uint16_t)msb << 8) | lsb;
    
    return true;
}