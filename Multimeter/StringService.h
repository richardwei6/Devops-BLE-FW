// Based off the HealthThermometerService

#ifndef __STRING_SERVICE_H__
#define __STRING_SERVICE_H__

#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/GattServer.h"

/**
* Base class that implements a BLE service with data type variable utf8s.
*/
template <size_t BufferSize>
class StringService {
public:
    StringService(BLE &ble, UUID charUuid, UUID serviceUuid) :
        ble_(ble),
        value_(charUuid, (uint8_t*)&buffer_, 0, BufferSize,
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE) {

        GattCharacteristic *chars[] = {&value_, };
        GattService service(serviceUuid, chars, sizeof(chars) / sizeof(chars[0]));

        ble.gattServer().addService(service);
    }

    void writeValue(const char* string) {
      size_t length = strlen(string);
      if (length >= BufferSize) {
        length = BufferSize - 1;  // account for null terminator
      }
      memcpy(buffer_, string, length);
      buffer_[length] = 0;

      ble_.gattServer().write(value_.getValueHandle(), reinterpret_cast<uint8_t *>(buffer_), length);
    }

protected:
    BLE &ble_;
    char buffer_[BufferSize] = {0};
    GattCharacteristic value_;
};

#endif // __STRING_SERVICE_H__
