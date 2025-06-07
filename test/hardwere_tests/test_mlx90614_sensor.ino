// Test for MLX90614 IR temperature sensor
// This code initializes the MLX90614 sensor and reads ambient and object temperatures.
// Verdict: All connections work.
// Connected pins: VCC => 3.3V; GND => GND; SDA => A4; SCL => A5 (for Arduino Uno)

#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!mlx.begin()) {
    Serial.println("MLX90614 not found. Check wiring!");
    while (1);
  }

  Serial.println("MLX90614 online!");
}

void loop() {
  Serial.print("Ambient = ");
  Serial.print(mlx.readAmbientTempC());
  Serial.print(" °C\tObject = ");
  Serial.print(mlx.readObjectTempC());
  Serial.println(" °C");

  delay(1000);
}

/*
Output example:
Ambient = 25.00 °C	Object = 26.50 °C
Ambient = 25.00 °C	Object = 26.50 °C
Ambient = 25.00 °C	Object = 26.50 °C
Ambient = 25.00 °C	Object = 26.50 °C
*/