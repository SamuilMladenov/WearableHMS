#include "MPU6050Sensor.h"

MPU6050::MPU6050() : _address(0x68), _initialized(false) {
}

bool MPU6050::begin(uint8_t address) {
    _address = address;
    
    Wire.begin();
    Wire.setClock(400000); // 400kHz for MPU6050
    
    delay(100);
    
    // Wake up MPU6050
    if (!writeRegister(0x6B, 0x00)) return false; // PWR_MGMT_1 register
    delay(100);
    
    // Verify connection
    uint8_t whoami;
    if (!readRegister(0x75, whoami)) return false; // WHO_AM_I register
    if (whoami != 0x68 && whoami != 0x72) return false; // Valid MPU6050 IDs
    
    // Configure accelerometer (+/- 2g)
    if (!writeRegister(0x1C, 0x00)) return false; // ACCEL_CONFIG
    
    // Configure gyroscope (+/- 250 deg/s)
    if (!writeRegister(0x1B, 0x00)) return false; // GYRO_CONFIG
    
    _initialized = true;
    return true;
}

bool MPU6050::readMotion(float &accelX, float &accelY, float &accelZ, 
                          float &gyroX, float &gyroY, float &gyroZ) {
    if (!_initialized) return false;
    
    Wire.beginTransmission(_address);
    Wire.write(0x3B); // ACCEL_XOUT_H register
    if (Wire.endTransmission(false) != 0) return false;
    
    Wire.requestFrom(_address, (uint8_t)14);
    if (Wire.available() < 14) return false;
    
    // Read accelerometer data
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    
    // Skip temperature (2 bytes)
    Wire.read();
    Wire.read();
    
    // Read gyroscope data
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();
    
    // Convert to g and deg/s
    accelX = ax / 16384.0; // +/- 2g range
    accelY = ay / 16384.0;
    accelZ = az / 16384.0;
    
    gyroX = gx / 131.0; // +/- 250 deg/s range
    gyroY = gy / 131.0;
    gyroZ = gz / 131.0;
    
    return true;
}

void MPU6050::printDiagnostics() {
    if (!_initialized) {
        Serial.println("MPU6050: Not initialized");
        return;
    }
    
    float ax, ay, az, gx, gy, gz;
    if (readMotion(ax, ay, az, gx, gy, gz)) {
        Serial.print("MPU6050: Accel=(");
        Serial.print(ax); Serial.print(",");
        Serial.print(ay); Serial.print(",");
        Serial.print(az); Serial.print(") Gyro=(");
        Serial.print(gx); Serial.print(",");
        Serial.print(gy); Serial.print(",");
        Serial.print(gz); Serial.println(")");
    } else {
        Serial.println("MPU6050: Read error");
    }
}

bool MPU6050::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool MPU6050::readRegister(uint8_t reg, uint8_t &value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    
    Wire.requestFrom(_address, (uint8_t)1);
    if (Wire.available() < 1) return false;
    
    value = Wire.read();
    return true;
}