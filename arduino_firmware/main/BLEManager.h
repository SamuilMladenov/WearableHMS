#pragma once
#include <Arduino.h>
#include <ArduinoBLE.h>

class BLEManager {
public:
  // Use a Nordic UART–style service/char for easy Android parsing
  // Service: 6E400001-...  TX Char (Notify): 6E400003-...
  BLEManager();

  // Start BLE + advertise
  bool begin(const char* deviceName);

  // Call frequently (non-blocking)
  void poll();

  // Send a JSON string via notifications (safe chunking)
  bool notifyJSON(const String& json);

  // For optional status checks
  bool isConnected() const { return BLE.connected(); }

private:
  BLEService _svc;
  BLECharacteristic _rx;  
  BLECharacteristic _tx;     

  // Chunks are kept conservative to work across phones/MTUs
  static const size_t kChunkSize = 20;

  bool _addAndAdvertise();
  void _sendInChunks(const uint8_t* data, size_t len);
};
