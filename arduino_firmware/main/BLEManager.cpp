#include "BLEManager.h"

// Nordic UART UUIDs
static const char* SVC_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* RX_UUID  = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // WRITE
static const char* TX_UUID  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // NOTIFY

BLEManager::BLEManager():
  _svc(SVC_UUID),
  _rx(RX_UUID, BLEWriteWithoutResponse, 20),
  _tx(TX_UUID, BLENotify, 20) {}


bool BLEManager::begin(const char* deviceName) {
  Serial.println("BLE.begin()...");
  if (!BLE.begin()) {
    Serial.println("BLE.begin FAILED");
    return false;
  }
  Serial.println("BLE.begin OK");

  BLE.setLocalName(deviceName);
  BLE.setDeviceName(deviceName);

  bool ok = _addAndAdvertise();
  Serial.print("Add+Advertise = ");
  Serial.println(ok ? "SUCCESS" : "FAIL");

  return ok;
}

bool BLEManager::_addAndAdvertise() {
  BLE.setAdvertisedService(_svc);

  _svc.addCharacteristic(_tx);
  BLE.addService(_svc);

  // Initial characteristic value so some Android stacks are happy
  _tx.writeValue((const uint8_t*)"", 0);

  BLE.advertise(); // Start advertising
  return true;
}



void BLEManager::poll() {
  BLE.poll();
}

bool BLEManager::notifyJSON(const String& json) {
  if (!BLE.connected()) {
    Serial.println("[BLE] Not connected");
    return false;
  }

  // Ensure newline
  String msg = json;
  if (!msg.endsWith("\n")) msg += "\n";

  const uint8_t* data = (const uint8_t*)msg.c_str();
  size_t len = msg.length();

  Serial.print("[BLE] Sending JSON length: ");
  Serial.println(len);

  _sendInChunks(data, len);   // do NOT return it

  return true;
}


void BLEManager::_sendInChunks(const uint8_t* data, size_t len) {
  size_t offset = 0;

  while (offset < len && BLE.connected()) {
    size_t chunkSize = 20;  // <-- HARD ENFORCE 20 BYTES
    if (offset + chunkSize > len)
        chunkSize = len - offset;

    _tx.writeValue(data + offset, chunkSize);

    Serial.print("[BLE] Sent chunk size=");
    Serial.println(chunkSize);

    offset += chunkSize;

    BLE.poll();
    delay(15);  // Give BLE stack time
  }

    Serial.println("[BLE] Finished sending chunks");
}


