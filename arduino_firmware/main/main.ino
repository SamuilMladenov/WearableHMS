#include "ADPDSensor.h"
#include "MLX90614Sensor.h"
#include "MPU6050Sensor.h"
#include "GSR.h"
#include "HeartRateCalculator.h"
#include "SpO2Calculator.h"
#include <ArduinoBLE.h>
#include "BLEManager.h"

// Sensor objects
ADPD105 heartSensor;
MLX90614 thermoSensor;
MPU6050 gyroSensor;
GSR gsrSensor;
BLEManager bleManager;

// Calculators
HeartRateCalculator hrCalc(500);
SpO2Calculator spo2Calc;

// State machine
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

// Timing constants
const unsigned long HEART_DURATION = 40000;
const unsigned long SPO2_DURATION  = 10000;
const unsigned long TEMP_DURATION  = 3000;
const unsigned long GYRO_DURATION  = 3000;
const unsigned long GSR_DURATION   = 3000;
const unsigned long IDLE_DURATION  = 31000;

// Results
float lastBPM = 0;
float lastSpO2 = 0;
float lastTemperature = 0;
float lastGSR = 0;
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;
float lastGyroX = 0, lastGyroY = 0, lastGyroZ = 0;
bool dataReady = false;
bool headerPrinted = false;
float lastStressLevel = 0;


// HRV metrics
float lastHRV_RMSSD = 0;
float lastHRV_SDNN = 0;
float lastHRV_pNN50 = 0;

// Forward declarations
void transitionToNextState();
void handleHeartState(unsigned long elapsed);
void handleSpO2State(unsigned long elapsed);
void handleTempState(unsigned long elapsed);
void handleGyroState(unsigned long elapsed);
void handleGSRState(unsigned long elapsed);
void handleIdleState(unsigned long elapsed);
void waitForUserStillness();

void setup() {
  delay(2000);
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Initializing Wearable HMS...");
  Serial.println("Sensors will run sequentially to avoid I2C conflicts.\n");

  Wire.begin();          
  Wire.setClock(100000);
  delay(20);

  // Initialize the MPU6050 first, since rest detection depends on it
  Serial.println("Initializing motion sensor...");
  if (!gyroSensor.begin(0x68)) {
    if (!gyroSensor.begin(0x69)) {
      Serial.println("ERROR: MPU6050 initialization failed!");
    }
  }

  // Start BLE (device name appears in Android scan)
  if (!ble.begin("WearableHMS")) {
    Serial.println("ERROR: BLE init failed!");
  } else {
    Serial.println("BLE advertising as 'WearableHMS'");
  }

  // Wait for the user to be still before starting the measurement sequence
  waitForUserStillness();

  stateStartTime = millis();
}

void loop() {
  ble.poll();

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - stateStartTime;

  switch (currentState) {
    case STATE_HEART: handleHeartState(elapsedTime); break;
    case STATE_SPO2:  handleSpO2State(elapsedTime);  break;
    case STATE_TEMP:  handleTempState(elapsedTime);  break;
    case STATE_GYRO:  handleGyroState(elapsedTime);  break;
    case STATE_GSR:   handleGSRState(elapsedTime);   break;
    case STATE_IDLE:  handleIdleState(elapsedTime);  break;
  }

  delay(10);
}

// Waits until the user is still for a few seconds before allowing measurement
void waitForUserStillness() {
  Serial.println("Waiting for user to be still...");
  unsigned long lastPrint = 0;

  while (true) {
    gyroSensor.updateMotionState();
    if (gyroSensor.isAtRest()) {
      Serial.println("User is at rest. Starting measurements...\n");
      break;
    }

    // Periodically print motion info for feedback
    unsigned long now = millis();
    if (now - lastPrint >= 1000) {
      Serial.print("Stillness check -> ΔAccRMS: ");
      Serial.print(gyroSensor.getAccelRMS(), 4);
      Serial.print(" g | ΔGyroRMS: ");
      Serial.print(gyroSensor.getGyroRMS(), 2);
      Serial.println(" °/s");
      
      lastPrint = now;
    }

    delay(200);
  }
}

void handleHeartState(unsigned long elapsed) {
  static bool initialized = false;
  static unsigned long nextTs = 0;
  const unsigned long SAMPLE_PERIOD_MS = 10;

  if (elapsed == 0 || !initialized) {
    Serial.println("[STATE] Reading Heart Rate for 40 seconds...");
    heartSensor.reset();
    delay(20);

    bool success = false;
    for (int i = 0; i < 3; i++) {
      if (heartSensor.begin(0x64, 100000)) { success = true; break; }
      Serial.print(" Retry "); Serial.println(i + 1);
      delay(150);
    }
    if (!success) {
      Serial.println("ERROR: Heart sensor initialization failed!");
      lastBPM = 0;
      transitionToNextState();
      return;
    }

    heartSensor.writeRegisterPublic(ADPD105::REG_FIFO_CLR, 0x0001);
    delay(5);
    heartSensor.writeRegisterPublic(ADPD105::REG_INT_STATUS, 0xFFFF);
    hrCalc.clearSamples();
    initialized = true;
    nextTs = millis();
  }

  uint8_t words = heartSensor.getFifoWordCount();

  if (words > 60) {
    Serial.println("[HR] FIFO near overflow -> resync");
    heartSensor.writeRegisterPublic(ADPD105::REG_MODE, 0x0000);
    delay(5);
    heartSensor.writeRegisterPublic(ADPD105::REG_FIFO_CLR, 0x0001);
    delay(5);
    heartSensor.writeRegisterPublic(ADPD105::REG_INT_STATUS, 0xFFFF);
    heartSensor.writeRegisterPublic(ADPD105::REG_MODE, 0x0002);
    return;
  }

  for (uint8_t i = 0; i < words; i++) {
    uint16_t sample;
    if (!heartSensor.readFifoData(sample)) break;
    if (sample >= 50 && sample <= 16380) {
      hrCalc.addSample(nextTs, sample);
      nextTs += SAMPLE_PERIOD_MS;
    }
  }

  if (elapsed >= HEART_DURATION) {
    hrCalc.printSampleStats();
    lastBPM = hrCalc.calculateBPM();
    Serial.print("[HEART] BPM: ");
    if (hrCalc.hasValidBPM()) {
      Serial.println(lastBPM, 1);

      // ➕ Calculate HRV metrics and store them
      auto hrv = hrCalc.calculateHRV();
      lastHRV_RMSSD = hrv.RMSSD;
      lastHRV_SDNN = hrv.SDNN;
      lastHRV_pNN50 = hrv.pNN50;

      Serial.print("[HRV] RMSSD: "); Serial.println(lastHRV_RMSSD, 1);
      Serial.print("[HRV] SDNN: ");  Serial.println(lastHRV_SDNN, 1);
      Serial.print("[HRV] pNN50: "); Serial.println(lastHRV_pNN50, 1);
    } 
    else {
      Serial.println("INVALID");
      lastBPM = 0;
      lastHRV_RMSSD = 0;
      lastHRV_SDNN = 0;
      lastHRV_pNN50 = 0;
    }

    initialized = false;
    transitionToNextState();
  }
}

void handleSpO2State(unsigned long elapsed) {
  static bool initialized = false;
  static unsigned long nextTs = 0;
  const unsigned long SAMPLE_PERIOD_MS = 10;

  if (elapsed == 0 || !initialized) {
    Serial.println("[STATE] Reading SpO2 for 10 seconds...");

    if (!heartSensor.begin(0x64, 100000, true)) {
      Serial.println("ERROR: SpO2 init failed!");
      lastSpO2 = 0;
      transitionToNextState();
      return;
    }

    heartSensor.writeRegisterPublic(ADPD105::REG_FIFO_CLR, 0x0001);
    delay(5);
    heartSensor.writeRegisterPublic(ADPD105::REG_INT_STATUS, 0xFFFF);

    spo2Calc.clear();
    initialized = true;
    nextTs = millis();
  }

  uint8_t words = heartSensor.getFifoWordCount();

  if (words > 60) {
    Serial.println("[SpO2] FIFO near overflow -> resync");
    heartSensor.writeRegisterPublic(ADPD105::REG_MODE, 0x0000);
    delay(5);
    heartSensor.writeRegisterPublic(ADPD105::REG_FIFO_CLR, 0x0001);
    delay(5);
    heartSensor.writeRegisterPublic(ADPD105::REG_INT_STATUS, 0xFFFF);
    heartSensor.writeRegisterPublic(ADPD105::REG_MODE, 0x0002);
    return;
  }

  uint8_t pairs = words / 2;
  for (uint8_t i = 0; i < pairs; i++) {
    uint16_t red, ir;
    if (!heartSensor.readFifoDataDual(red, ir)) break;
    if (red >= 50 && red <= 16380 && ir >= 50 && ir <= 16380) {
      spo2Calc.addSample(red, ir);
      nextTs += SAMPLE_PERIOD_MS;
    }
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    heartSensor.printDiagnostics();
    lastPrint = millis();
  }

  if (elapsed >= SPO2_DURATION) {
    lastSpO2 = spo2Calc.calculateSpO2();
    Serial.print("[SPO2] Estimated: ");
    Serial.print(lastSpO2, 1);
    Serial.println(" %");
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

    bool success = false;
    uint8_t addresses[] = {0x5A, 0x5B};
    for (uint8_t addr : addresses) {
      if (thermoSensor.begin(addr)) {
        success = true;
        Serial.print("  Found MLX90614 at 0x");
        Serial.println(addr, HEX);
        break;
      }
    }

    if (!success) {
      Serial.println("ERROR: Temperature sensor init failed!");
      lastTemperature = -1;
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
    lastTemperature = (tempCount > 0) ? (tempSum / tempCount) : -1;
    Serial.print("[TEMP] Average: ");
    Serial.print(lastTemperature);
    Serial.println(" °C");
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
  
    if (!gyroSensor.begin(0x68)) {
      if (!gyroSensor.begin(0x69)) {
        Serial.println("ERROR: Gyro sensor initialization failed!");
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
      lastGyroX = sumGX / gyroCount;
      lastGyroY = sumGY / gyroCount;
      lastGyroZ = sumGZ / gyroCount;
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
    static unsigned long cycleID = 0;
    cycleID++;

    // Compute and display stress
    lastStressLevel = computeStressLevel(lastGSR, lastTemperature, lastHRV_SDNN);

    Serial.print("[STRESS] Level: ");
    if (lastStressLevel < 0) {
      Serial.println("N/A (incomplete data)");
    } else {
      Serial.print(lastStressLevel, 1);
      Serial.print(" / 100 -> ");
      if (lastStressLevel < 30) Serial.println("Relaxed");
      else if (lastStressLevel < 60) Serial.println("Mild stress");
      else if (lastStressLevel < 85) Serial.println("High stress");
      else Serial.println("Extreme stress");
    }

    // JSON Output
    String json = "{";
    json += "\"cycle_id\":" + String(cycleID) + ",";
    json += "\"timestamp\":" + String(millis()) + ",";
    json += "\"bpm\":" + String(lastBPM, 6) + ",";
    json += "\"spo2\":" + String(lastSpO2, 6) + ",";
    json += "\"temp\":" + String(lastTemperature, 6) + ",";
    json += "\"gsr\":" + String(lastGSR, 6) + ",";
    json += "\"hrv_rmssd\":" + String(lastHRV_RMSSD, 6) + ",";
    json += "\"hrv_sdnn\":" + String(lastHRV_SDNN, 6) + ",";
    json += "\"hrv_pnn50\":" + String(lastHRV_pNN50, 6) + ",";
    json += "\"stress\":" + String(lastStressLevel, 6) + ",";
    json += "\"ax\":" + String(lastAccelX, 6) + ",";
    json += "\"ay\":" + String(lastAccelY, 6) + ",";
    json += "\"az\":" + String(lastAccelZ, 6) + ",";
    json += "\"gx\":" + String(lastGyroX, 6) + ",";
    json += "\"gy\":" + String(lastGyroY, 6) + ",";
    json += "\"gz\":" + String(lastGyroZ, 6);
    json += "}\\n";

    Serial.print(json);  // for now, print to Serial; later you’ll send this via BLE

    // Send to phone via BLE
    if (!ble.notifyJSON(json)) {
      Serial.println("[BLE]: BLE notify failed (not connected?)");
    }

    dataReady = false;
    outputPrinted = true;
}
  
  if (elapsed >= IDLE_DURATION) {
    Serial.println("\n[STATE] Starting new measurement cycle...\n");
    outputPrinted = false;

    // Before restarting, again check for stillness
    waitForUserStillness();
    transitionToNextState();
  }
}

void transitionToNextState() {
  stateStartTime = millis();
  
  switch (currentState) {
    case STATE_HEART: currentState = STATE_SPO2; break;
    case STATE_SPO2:  currentState = STATE_TEMP; break;
    case STATE_TEMP:  currentState = STATE_GYRO; break;
    case STATE_GYRO:  currentState = STATE_GSR;  break;
    case STATE_GSR:   currentState = STATE_IDLE; break;
    case STATE_IDLE:  currentState = STATE_HEART; break;
  }
}

float computeStressLevel(float gsr_ohm, float tempC, float hrvSDNN) {
  // 1) Handle invalid data 
  if (gsr_ohm <= 0 || gsr_ohm > 1e6 || tempC < 20 || tempC > 45 || hrvSDNN <= 0) {
    return -1; // Invalid / incomplete data
  }

  // 2) Normalize individual scores (0 = calm, 1 = stressed)
  // GSR: lower resistance => higher stress
  float gsrScore = (150000.0 - gsr_ohm) / 100000.0;
  gsrScore = constrain(gsrScore, 0.0, 1.0);

  // HRV: lower SDNN => higher stress
  float hrvScore = (50.0 - hrvSDNN) / 30.0;
  hrvScore = constrain(hrvScore, 0.0, 1.0);

  // Temperature: lower skin temp => higher stress
  float tempScore = (33.5 - tempC) / 3.0;
  tempScore = constrain(tempScore, 0.0, 1.0);

  // 3) Weighted sum 
  float stressLevel = (0.45 * gsrScore) + (0.40 * hrvScore) + (0.15 * tempScore);
  stressLevel = constrain(stressLevel * 100.0, 0.0, 100.0);

  return stressLevel;
}

