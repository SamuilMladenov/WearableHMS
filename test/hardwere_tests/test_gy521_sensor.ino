// Test code for MPU6050 (GY-521) sensor using Arduino
// This code initializes the MPU6050 sensor and reads raw gyroscope and accelerometer data.
// Verdict: All connections work.
// Connected pins: VCC => 3.3V; GND => GND; SDA => A4; SCL => A5 (for Arduino Uno)
// This code is used to test the connection of the MPU6050 sensor (GY-521).

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();

  Serial.println("Initializing MPU6050...");
  if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("MPU6050 not found. Check wiring!");
    while (1);
  }

  Serial.println("MPU6050 ready!");
}

void loop() {
  Vector rawGyro = mpu.readRawGyro();
  Vector rawAccel = mpu.readRawAccel();

  Serial.print("Gyro (X,Y,Z): ");
  Serial.print(rawGyro.XAxis); Serial.print(", ");
  Serial.print(rawGyro.YAxis); Serial.print(", ");
  Serial.println(rawGyro.ZAxis);

  Serial.print("Accel (X,Y,Z): ");
  Serial.print(rawAccel.XAxis); Serial.print(", ");
  Serial.print(rawAccel.YAxis); Serial.print(", ");
  Serial.println(rawAccel.ZAxis);

  delay(500);
}
/*
Output example:
Gyro (X,Y,Z): 0.00, 0.00, 0.00
Accel (X,Y,Z): 0.00, 0.00, 1.00
Gyro (X,Y,Z): 0.00, 0.00, 0.00
Accel (X,Y,Z): 0.00, 0.00, 1.00
*/ 