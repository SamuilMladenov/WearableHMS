#include "BLEManager.h"

// Nordic UART–style UUIDs (common & convenient)
static const char* SVC_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* TX_UUID  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

BLEManager::BLEManager()
: _svc(SVC_UUID),
  // Give a generous max length; we still chunk notifications manually.
  _tx(TX_UUID, BLERead | BLENotify, 244) {}

bool BLEManager::begin(const char* deviceName) {
  if (!BLE.begin()) {
    return false;
  }
  BLE.setLocalName(deviceName);
  BLE.setDeviceName(deviceName);
  return _addAndAdvertise();
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
  // Keep BLE stack happy; non-blocking
  BLE.poll();
}

bool BLEManager::notifyJSON(const String& json) {
  if (!BLE.connected()) return false;

  // Send in conservative chunks and use '\n' as message terminator 
  // Your Android app can concatenate until it sees '\n'.
  const uint8_t* bytes = (const uint8_t*)json.c_str();
  size_t len = json.length();

  _sendInChunks(bytes, len);
  return true;
}

void BLEManager::_sendInChunks(const uint8_t* data, size_t len) {
  size_t sent = 0;
  while (sent < len && BLE.connected()) {
    size_t n = min(kChunkSize, len - sent);

    // Update characteristic value buffer
    _tx.writeValue(data + sent, n);
    // Notify central
    _tx.notify();

    sent += n;
    // Give BLE time (very short yield)
    BLE.poll();
    delay(2);
  }
}
