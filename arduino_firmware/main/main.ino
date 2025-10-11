#include "ADPDSensor.h"
#include "MLX90614Sensor.h"
#include "MPU6050Sensor.h"
#include "GSR.h"

// Sensor objects
ADPD105 heartSensor;
MLX90614 thermoSensor;
MPU6050 gyroSensor;
GSR gsrSensor;

// Timing variables
unsigned long previousMillis = 0;
const long interval = 1000; // Read sensors every second

// ADPD105 variables
uint16_t heartSample = 0;
bool heartDataAvailable = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  // Initialize all sensors
  bool allSensorsOK = true;
  
  if (!heartSensor.begin()) {
    Serial.println("ADPD105 (Heart) sensor failed to initialize!");
    allSensorsOK = false;
  } else {
    Serial.println("ADPD105 (Heart) sensor initialized");
  }
  
  if (!thermoSensor.begin()) {
    Serial.println("MLX90614 (Thermo) sensor failed to initialize!");
    allSensorsOK = false;
  } else {
    Serial.println("MLX90614 (Thermo) sensor initialized");
  }
  
  if (!gyroSensor.begin()) {
    Serial.println("MPU6050 (Gyro) sensor failed to initialize!");
    allSensorsOK = false;
  } else {
    Serial.println("MPU6050 (Gyro) sensor initialized");
  }
  
  if (!gsrSensor.begin()) {
    Serial.println("GSR sensor failed to initialize!");
    allSensorsOK = false;
  } else {
    Serial.println("GSR sensor initialized");
  }
  
  Serial.println("\n=== Sensor Readings ===");
  Serial.println("Time(ms)\tHeart\tTemp(°C)\tGSR\tAccelX\tAccelY\tAccelZ\tGyroX\tGyroY\tGyroZ");
  Serial.println("--------------------------------------------------------------------------------");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Read ADPD105 more frequently since it has FIFO data
  heartDataAvailable = heartSensor.readFifoData(heartSample);
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Read other sensors
    float temperature = thermoSensor.readTemperature();
    float gsrValue = gsrSensor.readGSR();
    
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    gyroSensor.readMotion(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
    
    // Print readings in a structured format
    Serial.print(currentMillis);
    Serial.print("\t");
    
    // Heart rate data (raw ADPD105 sample)
    if (heartDataAvailable) {
      Serial.print(heartSample);
    } else {
      Serial.print("NO_DATA");
    }
    Serial.print("\t");
    
    // Temperature
    if (temperature < 0) Serial.print("ERROR");
    else Serial.print(temperature);
    Serial.print("\t");
    
    // GSR
    if (gsrValue < 0) Serial.print("ERROR");
    else Serial.print(gsrValue);
    Serial.print("\t");
    
    // Accelerometer
    Serial.print(accelX); Serial.print("\t");
    Serial.print(accelY); Serial.print("\t");
    Serial.print(accelZ); Serial.print("\t");
    
    // Gyroscope
    Serial.print(gyroX); Serial.print("\t");
    Serial.print(gyroY); Serial.print("\t");
    Serial.print(gyroZ);
    
    Serial.println();
    
    // Additional diagnostic info every 10 seconds
    if (currentMillis % 10000 == 0) {
      Serial.println("\n=== Sensor Diagnostics ===");
      heartSensor.printDiagnostics();
      thermoSensor.printDiagnostics();
      gyroSensor.printDiagnostics();
      gsrSensor.printDiagnostics();
      Serial.println("---------------------------\n");
    }
  }
  
  delay(10); // Light polling for ADPD105 FIFO
}