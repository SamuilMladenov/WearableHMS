#include <Wire.h>
#include "ADPDSensor.h"
#include "MLX90614Sensor.h"
#include "MPU6050Sensor.h"
#include "GSR.h"
#include "HeartRateCalculator.h"
#include "SpO2Calculator.h"

ADPD105   heartSensor;
MLX90614  thermoSensor;
MPU6050   gyroSensor;
GSR       gsrSensor;

HeartRateCalculator hrCalc(500);
SpO2Calculator      spo2Calc;

enum SensorState {
  STATE_HEART,
  STATE_SPO2,
  STATE_TEMP,
  STATE_GYRO,
  STATE_GSR,
  STATE_IDLE
};

SensorState currentState = STATE_HEART;
unsigned long stateStartTime = 0;

const unsigned long HEART_DURATION = 20000;
const unsigned long SPO2_DURATION  = 20000;
const unsigned long TEMP_DURATION  = 3000;
const unsigned long GYRO_DURATION  = 3000;
const unsigned long GSR_DURATION   = 3000;
const unsigned long IDLE_DURATION  = 11000;

float lastBPM = 0;
float lastSpO2 = 0;
float lastTemperature = 0;
float lastGSR = 0;
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;
float lastGyroX  = 0, lastGyroY  = 0, lastGyroZ  = 0;
bool  dataReady = false;
bool headerPrinted = false;

void transitionToNextState();
void handleHeartState(unsigned long elapsed);
void handleSpO2State(unsigned long elapsed);
void handleTempState(unsigned long elapsed);
void handleGyroState(unsigned long elapsed);
void handleGSRState(unsigned long elapsed);
void handleIdleState(unsigned long elapsed);

static inline void resetI2C() {
#if defined(TWCR) || defined(ESP8266) || defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
  Wire.end();
#endif
  delay(100);
  Wire.begin();
  delay(10);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Wire.begin();

  Serial.println("Initializing Wearable HMS...");
  Serial.println("Sensors will run sequentially to avoid I2C conflicts.");
  Serial.println();

  stateStartTime = millis();
}

void loop() {
  const unsigned long now = millis();
  const unsigned long elapsed = now - stateStartTime;

  switch (currentState) {
    case STATE_HEART: handleHeartState(elapsed); break;
    case STATE_SPO2:  handleSpO2State(elapsed);  break;
    case STATE_TEMP:  handleTempState(elapsed);  break;
    case STATE_GYRO:  handleGyroState(elapsed);  break;
    case STATE_GSR:   handleGSRState(elapsed);   break;
    case STATE_IDLE:  handleIdleState(elapsed);  break;
  }

  delay(10);
}

void handleHeartState(unsigned long elapsed) {
  static bool initialized = false;

  if (elapsed == 0 || !initialized) {
    Serial.println("[STATE] Reading Heart Rate for 20 seconds...");
    resetI2C();

    bool success = false;
    for (int i = 0; i < 3; i++) {
      if (heartSensor.begin(0x64, 100000)) {
        if (heartSensor.configureForHeartRate()) {
          success = true;
          break;
        }
      }
      Serial.print("  Retry ");
      Serial.println(i + 1);
      delay(200);
    }

    if (!success) {
      Serial.println("ERROR: Heart sensor initialization failed after 3 attempts!");
      lastBPM = 0;
      initialized = false;
      transitionToNextState();
      return;
    }

    hrCalc.clearSamples();
    initialized = true;
  }

  uint16_t sample;
  if (heartSensor.readFifoData(sample)) {
    hrCalc.addSample(millis(), sample);
  }

  if (elapsed >= HEART_DURATION) {
    hrCalc.printSampleStats();

    lastBPM = hrCalc.calculateBPM();
    Serial.print("[HEART] Collected ");
    Serial.print(hrCalc.getSampleCount());
    Serial.print(" samples, ");
    Serial.print(hrCalc.getPeakCount());
    Serial.print(" peaks detected, BPM: ");
    if (hrCalc.hasValidBPM()) {
      Serial.println(lastBPM);
    } else {
      Serial.println("INVALID");
      lastBPM = 0;
    }

    initialized = false;
    transitionToNextState();
  }
}

void handleSpO2State(unsigned long elapsed) {
  static bool initialized = false;

  if (elapsed == 0 || !initialized) {
    Serial.println("[STATE] Reading SpO2 for 20 seconds...");
    resetI2C();

    bool success = false;
    for (int i = 0; i < 3; i++) {
      if (heartSensor.begin(0x64, 100000)) {
        if (heartSensor.configureForSpO2()) {
          success = true;
          break;
        }
      }
      Serial.print("  Retry ");
      Serial.println(i + 1);
      delay(200);
    }

    if (!success) {
      Serial.println("ERROR: SpO2 sensor configuration failed after 3 attempts!");
      lastSpO2 = 0;
      initialized = false;
      transitionToNextState();
      return;
    }

    spo2Calc.clearSamples();
    initialized = true;
  }

  uint16_t redSample, irSample;
  if (heartSensor.readFifoDataDual(redSample, irSample)) {
    spo2Calc.addSample(millis(), redSample, irSample);
  }

  if (elapsed >= SPO2_DURATION) {
    spo2Calc.printSampleStats();

    lastSpO2 = spo2Calc.calculateSpO2();
    Serial.print("[SPO2] Collected ");
    Serial.print(spo2Calc.getSampleCount());
    Serial.print(" sample pairs, SpO2: ");
    if (spo2Calc.hasValidSpO2()) {
      Serial.print(lastSpO2);
      Serial.println("%");
    } else {
      Serial.println("INVALID");
      lastSpO2 = 0;
    }

    initialized = false;
    transitionToNextState();
  }
}

void handleTempState(unsigned long elapsed) {
  static bool initialized = false;
  static float tempSum = 0;
  static int tempCount = 0;

  if (elapsed == 0 || !initialized) {
    Serial.println("[STATE] Reading Temperature for 3 seconds...");
    resetI2C();

    bool success = false;
    uint8_t addresses[] = {0x5A, 0x5B};

    for (int a = 0; a < 2; a++) {
      for (int i = 0; i < 2; i++) {
        if (thermoSensor.begin(addresses[a])) {
          success = true;
          Serial.print("  Found at 0x");
          Serial.println(addresses[a], HEX);
          break;
        }
        delay(100);
      }
      if (success) break;
    }

    if (!success) {
      Serial.println("ERROR: Temperature sensor initialization failed!");
      lastTemperature = -1;
      initialized = false;
      transitionToNextState();
      return;
    }

    tempSum = 0;
    tempCount = 0;
    initialized = true;
  }

  static unsigned long lastTempRead = 0;
  if (millis() - lastTempRead >= 100) {
    lastTempRead = millis();
    float t = thermoSensor.readTemperature();
    if (t > -50 && t < 150) {
      tempSum += t;
      tempCount++;
    }
  }

  if (elapsed >= TEMP_DURATION) {
    if (tempCount > 0) {
      lastTemperature = tempSum / tempCount;
      Serial.print("[TEMP] Average: ");
      Serial.print(lastTemperature);
      Serial.println(" °C");
    } else {
      lastTemperature = -1;
      Serial.println("[TEMP] ERROR");
    }

    initialized = false;
    transitionToNextState();
  }
}

void handleGyroState(unsigned long elapsed) {
  static bool initialized = false;
  static float sumAX = 0, sumAY = 0, sumAZ = 0;
  static float sumGX = 0, sumGY = 0, sumGZ = 0;
  static int gyroCount = 0;

  if (elapsed == 0 || !initialized) {
    Serial.println("[STATE] Reading Gyroscope/Accelerometer for 3 seconds...");
    resetI2C();

    if (!gyroSensor.begin(0x68)) {
      if (!gyroSensor.begin(0x69)) {
        Serial.println("ERROR: Gyro sensor initialization failed!");
        initialized = false;
        transitionToNextState();
        return;
      }
    }

    sumAX = sumAY = sumAZ = 0;
    sumGX = sumGY = sumGZ = 0;
    gyroCount = 0;
    initialized = true;
  }

  static unsigned long lastGyroRead = 0;
  if (millis() - lastGyroRead >= 100) {
    lastGyroRead = millis();
    float ax, ay, az, gx, gy, gz;
    if (gyroSensor.readMotion(ax, ay, az, gx, gy, gz)) {
      sumAX += ax; sumAY += ay; sumAZ += az;
      sumGX += gx; sumGY += gy; sumGZ += gz;
      gyroCount++;
    }
  }

  if (elapsed >= GYRO_DURATION) {
    if (gyroCount > 0) {
      lastAccelX = sumAX / gyroCount;
      lastAccelY = sumAY / gyroCount;
      lastAccelZ = sumAZ / gyroCount;
      lastGyroX  = sumGX / gyroCount;
      lastGyroY  = sumGY / gyroCount;
      lastGyroZ  = sumGZ / gyroCount;
      Serial.println("[GYRO] Data collected");
    } else {
      Serial.println("[GYRO] ERROR");
    }

    initialized = false;
    transitionToNextState();
  }
}

void handleGSRState(unsigned long elapsed) {
  static bool initialized = false;
  static float gsrSum = 0;
  static int gsrCount = 0;

  if (elapsed == 0 || !initialized) {
    Serial.println("[STATE] Reading GSR for 3 seconds...");

    uint8_t pins[] = {A0, A1, A2, A3};
    bool success = false;

    for (int i = 0; i < 4; i++) {
      if (gsrSensor.begin(pins[i])) {
        Serial.print("  GSR on pin A");
        Serial.println(pins[i] - A0);
        float testValue = gsrSensor.readGSR();
        if (testValue > 0) {
          success = true;
          break;
        }
      }
    }

    if (!success) {
      Serial.println("ERROR: GSR sensor initialization failed!");
      lastGSR = -1;
      initialized = false;
      transitionToNextState();
      return;
    }

    gsrSum = 0;
    gsrCount = 0;
    initialized = true;
  }

  static unsigned long lastGSRRead = 0;
  if (millis() - lastGSRRead >= 100) {
    lastGSRRead = millis();
    float gsr = gsrSensor.readGSR();
    if (gsr > 0 && gsr < 999999.0) {
      gsrSum += gsr;
      gsrCount++;
    }
  }

  if (elapsed >= GSR_DURATION) {
    if (gsrCount > 0) {
      lastGSR = gsrSum / gsrCount;
      Serial.print("[GSR] Average: ");
      Serial.println(lastGSR);
    } else {
      lastGSR = -1;
      Serial.println("[GSR] ERROR");
    }

    initialized = false;
    dataReady = true;
    transitionToNextState();
  }
}

void handleIdleState(unsigned long elapsed) {
  static bool outputPrinted = false;

  if (!outputPrinted && dataReady) {
    if (!headerPrinted) {
      Serial.println("\n=== CSV OUTPUT ===");
      Serial.println("Timestamp,BPM,SpO2,Temperature_C,GSR,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");
      headerPrinted = true;
    }

    Serial.print(millis());            Serial.print(",");
    Serial.print(lastBPM, 1);         Serial.print(",");
    Serial.print(lastSpO2, 1);        Serial.print(",");
    Serial.print(lastTemperature, 2); Serial.print(",");
    Serial.print(lastGSR, 2);         Serial.print(",");
    Serial.print(lastAccelX, 2);      Serial.print(",");
    Serial.print(lastAccelY, 2);      Serial.print(",");
    Serial.print(lastAccelZ, 2);      Serial.print(",");
    Serial.print(lastGyroX, 2);       Serial.print(",");
    Serial.print(lastGyroY, 2);       Serial.print(",");
    Serial.println(lastGyroZ, 2);

    dataReady = false;
    outputPrinted = true;
  }

  if (elapsed >= IDLE_DURATION) {
    Serial.println("\n[STATE] Starting new measurement cycle...\n");
    outputPrinted = false;
    transitionToNextState();
  }
}

void transitionToNextState() {
  stateStartTime = millis();

  switch (currentState) {
    case STATE_HEART: currentState = STATE_SPO2;  break;
    case STATE_SPO2:  currentState = STATE_TEMP;  break;
    case STATE_TEMP:  currentState = STATE_GYRO;  break;
    case STATE_GYRO:  currentState = STATE_GSR;   break;
    case STATE_GSR:   currentState = STATE_IDLE;  break;
    case STATE_IDLE:  currentState = STATE_HEART; break;
  }
}
