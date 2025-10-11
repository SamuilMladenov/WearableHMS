#include "MPU6050Sensor.h"

MPU6050::MPU6050() : _address(0x68), _initialized(false) {}

bool MPU6050::begin(uint8_t address) {
    _address = address;
    Wire.begin();
    
    if (!checkConnection()) {
        return false;
    }
    
    initializeSensor();
    _initialized = true;
    return true;
}

bool MPU6050::checkConnection() {
    Wire.beginTransmission(_address);
    Wire.write(0x75); // WHO_AM_I register
    Wire.endTransmission(false);
    Wire.requestFrom(_address, (uint8_t)1);
    
    if (Wire.available()) {
        uint8_t whoami = Wire.read();
        return (whoami == 0x68); // MPU6050 should return 0x68
    }
    return false;
}

void MPU6050::initializeSensor() {
    // Wake up the sensor
    Wire.beginTransmission(_address);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0x00); // Set to zero to wake up
    Wire.endTransmission();
    
    // Configure accelerometer range ±2g
    Wire.beginTransmission(_address);
    Wire.write(0x1C); // ACCEL_CONFIG register
    Wire.write(0x00); // ±2g range
    Wire.endTransmission();
    
    // Configure gyroscope range ±250°/s
    Wire.beginTransmission(_address);
    Wire.write(0x1B); // GYRO_CONFIG register
    Wire.write(0x00); // ±250°/s range
    Wire.endTransmission();
}

void MPU6050::readMotion(float &accelX, float &accelY, float &accelZ, 
                        float &gyroX, float &gyroY, float &gyroZ) {
    if (!_initialized) {
        accelX = accelY = accelZ = gyroX = gyroY = gyroZ = 0.0;
        return;
    }
    
    Wire.beginTransmission(_address);
    Wire.write(0x3B); // Starting register for accelerometer data
    Wire.endTransmission(false);
    Wire.requestFrom(_address, (uint8_t)14);
    
    if (Wire.available() >= 14) {
        // Read accelerometer data
        int16_t ax = (Wire.read() << 8) | Wire.read();
        int16_t ay = (Wire.read() << 8) | Wire.read();
        int16_t az = (Wire.read() << 8) | Wire.read();
        
        // Read temperature (skip)
        Wire.read(); Wire.read();
        
        // Read gyroscope data
        int16_t gx = (Wire.read() << 8) | Wire.read();
        int16_t gy = (Wire.read() << 8) | Wire.read();
        int16_t gz = (Wire.read() << 8) | Wire.read();
        
        // Convert to meaningful values
        accelX = ax / 16384.0; // ±2g range
        accelY = ay / 16384.0;
        accelZ = az / 16384.0;
        
        gyroX = gx / 131.0; // ±250°/s range
        gyroY = gy / 131.0;
        gyroZ = gz / 131.0;
    }
}

void MPU6050::printDiagnostics() {
    Serial.println("MPU6050 Diagnostics:");
    Serial.print("  Connected: "); Serial.println(checkConnection() ? "Yes" : "No");
    Serial.print("  Initialized: "); Serial.println(_initialized ? "Yes" : "No");
    Serial.print("  I2C Address: 0x"); Serial.println(_address, HEX);
}