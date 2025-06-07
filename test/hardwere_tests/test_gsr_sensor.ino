// This code is used to test the connection of the GSR sensor.
// Verdict: All conections work.
// Connected pins: VCC => 3.3V; GND=>GND; SIG=> A0 (Can be connected to any analog pin)
const int GSR_PIN = A0;  // Analog input for GSR
int sensorValue = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Grove GSR Sensor Test");
}

void loop() {
  sensorValue = analogRead(GSR_PIN);

  Serial.print("GSR analog value: ");
  Serial.println(sensorValue);

  delay(200);
}

/*
Output:
GSR analog value: 512
GSR analog value: 511
GSR analog value: 513
GSR analog value: 512
GSR analog value: 512
GSR analog value: 507
*/
