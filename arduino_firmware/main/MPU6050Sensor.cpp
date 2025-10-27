#include "MPU6050Sensor.h"

MPU6050::MPU6050()
    : _address(0x68), _initialized(false),
      _accRMS(0), _gyroRMS(0), _lastUpdate(0),
      _currentlyRest(false), _restStartTime(0) {}

bool MPU6050::begin(uint8_t address) {
    _address = address;
    Wire.begin();
    Wire.setClock(400000);
    delay(100);

    // Wake up the MPU6050 (exit sleep mode)
    if (!writeRegister(0x6B, 0x00)) return false;
    delay(50);

    // Verify connection
    uint8_t whoami;
    if (!readRegister(0x75, whoami)) return false;
    if (whoami != 0x68 && whoami != 0x72) return false;

    // Configure ±2 g and ±250 °/s ranges
    if (!writeRegister(0x1C, 0x00)) return false; // ACCEL_CONFIG
    if (!writeRegister(0x1B, 0x00)) return false; // GYRO_CONFIG

    _initialized = true;
    return true;
}

bool MPU6050::readMotion(float &accelX, float &accelY, float &accelZ,
                         float &gyroX, float &gyroY, float &gyroZ) {
    if (!_initialized) return false;

    Wire.beginTransmission(_address);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom(_address, (uint8_t)14);
    if (Wire.available() < 14) return false;

    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // skip temperature
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();

    accelX = ax / 16384.0;
    accelY = ay / 16384.0;
    accelZ = az / 16384.0;
    gyroX  = gx / 131.0;
    gyroY  = gy / 131.0;
    gyroZ  = gz / 131.0;

    return true;
}

void MPU6050::printDiagnostics() {
    if (!_initialized) {
        Serial.println("MPU6050: Not initialized");
        return;
    }
    float ax, ay, az, gx, gy, gz;
    if (readMotion(ax, ay, az, gx, gy, gz)) {
        Serial.print("Accel=(");
        Serial.print(ax, 2); Serial.print(", ");
        Serial.print(ay, 2); Serial.print(", ");
        Serial.print(az, 2); Serial.print(") Gyro=(");
        Serial.print(gx, 1); Serial.print(", ");
        Serial.print(gy, 1); Serial.print(", ");
        Serial.print(gz, 1); Serial.println(")");
    } else {
        Serial.println("MPU6050: Read error");
    }
}

void MPU6050::updateMotionState() {
    if (!_initialized) return;

    float ax, ay, az, gx, gy, gz;
    if (!readMotion(ax, ay, az, gx, gy, gz)) return;

    static float lastAx = 0, lastAy = 0, lastAz = 0;
    static float lastGx = 0, lastGy = 0, lastGz = 0;
    static bool first = true;

    // Compute the change (delta) since the last reading
    float dax = ax - lastAx;
    float day = ay - lastAy;
    float daz = az - lastAz;
    float dgx = gx - lastGx;
    float dgy = gy - lastGy;
    float dgz = gz - lastGz;

    if (!first) {
        // Compute RMS of differences (true motion)
        float accDelta = sqrtf(dax * dax + day * day + daz * daz);
        float gyroDelta = sqrtf(dgx * dgx + dgy * dgy + dgz * dgz);

        // Exponential smoothing for stability
        const float alpha = 0.2f;
        _accRMS  = alpha * accDelta  + (1 - alpha) * _accRMS;
        _gyroRMS = alpha * gyroDelta + (1 - alpha) * _gyroRMS;
    } else {
        first = false;
        _accRMS = _gyroRMS = 0;
    }

    lastAx = ax; lastAy = ay; lastAz = az;
    lastGx = gx; lastGy = gy; lastGz = gz;

    unsigned long now = millis();
    bool below = (_accRMS < ACC_REST_THRESHOLD && _gyroRMS < GYRO_REST_THRESHOLD);

    if (below) {
        if (!_currentlyRest) {
            _currentlyRest = true;
            _restStartTime = now;
        }
    } else {
        _currentlyRest = false;
        _restStartTime = 0;
    }

    _lastUpdate = now;
}


bool MPU6050::isAtRest() {
    // Update should be called regularly before this
    if (!_initialized) return false;

    unsigned long now = millis();

    // Still for enough consecutive time
    if (_currentlyRest && (now - _restStartTime >= REST_STABLE_TIME))
        return true;

    return false;
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
    if (!Wire.available()) return false;
    value = Wire.read();
    return true;
}
