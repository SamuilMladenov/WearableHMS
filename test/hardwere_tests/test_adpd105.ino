#include <Arduino.h>
#include <Wire.h>

#define ADPD105_ADDR 0x64

// Registers
#define REG_STATUS       0x00
#define REG_SW_RESET     0x0F
#define REG_MODE         0x10
#define REG_SLOT_EN      0x11
#define REG_FSAMPLE      0x12
#define REG_PD_LED_SEL   0x14
#define REG_NUM_AVG      0x15
#define REG_LED1_DRV     0x23
#define REG_LED2_DRV     0x24
#define REG_CLK32K       0x4B
#define REG_FIFO_DATA    0x60

// Filters and peak detection
struct ExpAvg {
  float y = 0, a;
  ExpAvg(float alpha) : a(alpha) {}
  float step(float x) { y = a * y + (1 - a) * x; return y; }
};

ExpAvg dcRed(0.995f), dcIR(0.995f), envHR(0.90f), bpmSmooth(0.90f);
float lastAC = 0.0f;
uint32_t lastPeakMs = 0;
float lastBPM = NAN;

// Tuning
const float THRESH_FRAC = 0.35f;
const uint16_t IBI_MIN_MS = 300;
const uint16_t IBI_MAX_MS = 2000;

// I2C helpers
bool wr16(uint8_t r, uint16_t v) {
  Wire.beginTransmission(ADPD105_ADDR);
  Wire.write(r);
  Wire.write(uint8_t(v >> 8));
  Wire.write(uint8_t(v & 0xFF));
  return Wire.endTransmission() == 0;
}

bool rd16(uint8_t r, uint16_t &o) {
  Wire.beginTransmission(ADPD105_ADDR);
  Wire.write(r);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ADPD105_ADDR, (uint8_t)2) != 2) return false;
  o = (uint16_t(Wire.read()) << 8) | Wire.read();
  return true;
}

bool rdFifoWords(uint8_t n, uint16_t *buf) {
  Wire.beginTransmission(ADPD105_ADDR);
  Wire.write(REG_FIFO_DATA);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ADPD105_ADDR, (uint8_t)(2 * n)) != 2 * n) return false;
  for (uint8_t i = 0; i < n; i++)
    buf[i] = (uint16_t(Wire.read()) << 8) | Wire.read();
  return true;
}

// Config both LEDs on (dual-slot)
bool configDual() {
  if (!wr16(REG_CLK32K, 0x4C92)) return false;
  if (!wr16(REG_MODE, 0x0001)) return false;   // program mode
  if (!wr16(REG_FSAMPLE, 0x0050)) return false; // 100 Hz
  if (!wr16(REG_NUM_AVG, 0x0000)) return false;

  // PD/LED mapping: A->LED1(RED), B->LED2(IR)
  if (!wr16(REG_PD_LED_SEL, 0x0559)) return false;

  // LED currents — tweak these if needed
  if (!wr16(REG_LED1_DRV, 0x2018)) return false; // RED
  if (!wr16(REG_LED2_DRV, 0x2018)) return false; // IR

  // Enable A + B slots
  if (!wr16(REG_SLOT_EN, 0x1065)) return false;

  return wr16(REG_MODE, 0x0002); // NORMAL mode (start)
}

// Process combined Red + IR to detect BPM
void processSample(uint16_t redRaw, uint16_t irRaw) {
  float red = redRaw, ir = irRaw;
  float redDC = dcRed.step(red);
  float irDC = dcIR.step(ir);
  float combinedAC = ((red - redDC) + (ir - irDC)) / 2.0f;

  float envNow = envHR.step(fabsf(combinedAC));
  float thr = THRESH_FRAC * envNow;

  bool upCross = (lastAC < thr) && (combinedAC >= thr);
  lastAC = combinedAC;

  if (upCross) {
    uint32_t now = millis();
    if (lastPeakMs != 0) {
      uint32_t ibi = now - lastPeakMs;
      if (ibi >= IBI_MIN_MS && ibi <= IBI_MAX_MS) {
        float bpm = 60000.0f / (float)ibi;
        lastBPM = bpmSmooth.step(bpm);
      }
    }
    lastPeakMs = now;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Wire.begin();
  Wire.setClock(100000);

  wr16(REG_SW_RESET, 0x0001);
  delay(5);

  if (!configDual()) {
    Serial.println("Failed to configure ADPD105!");
    while (1);
  }

  Serial.println("ADPD105 Dual LED Heart Rate Mode (RED + IR)");
}

void loop() {
  uint16_t st = 0;
  if (!rd16(REG_STATUS, st)) { delay(2); return; }
  uint8_t bytesAvail = uint8_t(st >> 8);

  while (bytesAvail >= 4) { // 2 words (A,B)
    uint16_t ab[2];
    if (!rdFifoWords(2, ab)) break; // ab[0]=RED, ab[1]=IR
    processSample(ab[0], ab[1]);
    if (!rd16(REG_STATUS, st)) break;
    bytesAvail = uint8_t(st >> 8);
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 5000) { // print every 5s
    lastPrint = millis();
    Serial.print("BPM: ");
    if (!isnan(lastBPM)) Serial.println((int)(lastBPM + 0.5f));
    else Serial.println("---");
  }

  delay(2);
}
