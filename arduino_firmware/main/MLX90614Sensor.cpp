#include "MLX90614Sensor.h"

MLX90614::MLX90614() : _address(0x5A), _initialized(false) {}

bool MLX90614::begin(uint8_t address) {
    _address = address;
    Wire.begin();
    
    if (!checkConnection()) {
        return false;
    }
    
    _initialized = true;
    return true;
}

bool MLX90614::checkConnection() {
    Wire.beginTransmission(_address);
    return (Wire.endTransmission() == 0);
}

float MLX90614::readTemperature() {
    if (!_initialized) return -1.0;
    
    return readTemp(0x07); // Read object temperature
}

float MLX90614::readTemp(uint8_t reg) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.endTransmission(false);
    
    Wire.requestFrom(_address, (uint8_t)3);
    
    if (Wire.available() >= 3) {
        uint8_t dataLow = Wire.read();
        uint8_t dataHigh = Wire.read();
        Wire.read(); // Read PEC (optional)
        
        uint16_t tempData = (dataHigh << 8) | dataLow;
        float temperature = (tempData * 0.02) - 273.15;
        return temperature;
    }
    
    return -1.0;
}

void MLX90614::printDiagnostics() {
    Serial.println("MLX90614 Diagnostics:");
    Serial.print("  Connected: "); Serial.println(checkConnection() ? "Yes" : "No");
    Serial.print("  Initialized: "); Serial.println(_initialized ? "Yes" : "No");
    Serial.print("  I2C Address: 0x"); Serial.println(_address, HEX);
    
    if (_initialized) {
        float temp = readTemperature();
        Serial.print("  Current Temp: "); Serial.print(temp); Serial.println(" °C");
    }
}