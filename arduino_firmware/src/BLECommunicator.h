#ifndef BLE_COMMUNICATOR_H
#define BLE_COMMUNICATOR_H

#include <ArduinoBLE.h>
#include "HealthData.h"

class BLECommunicator {
public:
    BLECommunicator() {}
    bool begin();
    void sendData(const HealthData &data);

private:
    // Define custom service and characteristic UUIDs
    const char *serviceUuid = "12345678-1234-5678-1234-56789ABCDEF0"; //may need change
    const char *charUuid    = "12345678-1234-5678-1234-56789ABCDEF1"; //may need change

    // BLE Service and Characteristic objects
    BLEService healthService{serviceUuid};
    BLECharacteristic dataChar{charUuid, BLENotify | BLERead, 100};
};

#endif // BLE_COMMUNICATOR_H
