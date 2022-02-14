#ifndef __NUS_SERVICE_H__
#define __NUS_SERVICE_H__

#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/GattServer.h"


/** Nordic UART Service (NUS), a non-standard BLE service that presents a UART bridge.
 * Based on https://github.com/ARMmbed/ble/blob/master/ble/services/UARTService.h,
 * which is no longer included in recent mbed releases.
 * 
 * For whatever reason, this doesn't seem to work well with nRF Toolbox, but does work with
 * Serial Bluetooth Terminal
 */
class NusService {
public:
  NusService(BLE &ble) : ble_(ble),
      rxCharacteristic_(kRxCharacteristicUuid, outBuffer_, 0, kMaxBufferLen,
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),
      txCharacteristic_(kTxCharacteristicUuid, inBuffer_, 0, kMaxBufferLen,
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE) {
    GattCharacteristic *charTable[] = {&rxCharacteristic_, &txCharacteristic_};
    GattService nusService(kServiceUuid, charTable, sizeof(charTable) / sizeof(charTable[0]));
    ble.gattServer().addService(nusService);
  }

  size_t write(const uint8_t *data, size_t length) {
    if (length >= kMaxBufferLen) {
      length = kMaxBufferLen - 1;  // account for null terminator (?)
    }
    memcpy(outBuffer_, data, length + 1);
    outBuffer_[length] = 0;
    ble_.gattServer().write(rxCharacteristic_.getValueHandle(), outBuffer_, length);

    return length;
  }

  size_t write(char* data) {
    return write((const uint8_t*)data, strlen(data));
  }


  // from https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v14.0.0%2Fble_sdk_app_nus_eval.html
  static constexpr uint8_t kServiceUuid[UUID::LENGTH_OF_LONG_UUID] = {
    0x6E, 0x40, 0x00, 0x01,
    0xB5, 0xA3,
    0xF3, 0x93,
    0xE0, 0xA9,
    0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E
  };
  static constexpr uint8_t kRxCharacteristicUuid[UUID::LENGTH_OF_LONG_UUID] = {
    0x6E, 0x40, 0x00, 0x02,
    0xB5, 0xA3,
    0xF3, 0x93,
    0xE0, 0xA9,
    0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E
  };
  static constexpr uint8_t kTxCharacteristicUuid[UUID::LENGTH_OF_LONG_UUID] = {
    0x6E, 0x40, 0x00, 0x03,
    0xB5, 0xA3,
    0xF3, 0x93,
    0xE0, 0xA9,
    0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E
  };

protected:
  BLE& ble_;
  GattCharacteristic rxCharacteristic_;  // for outbound data (rx from external client)
  GattCharacteristic txCharacteristic_;  // for inbound data (tx from external client)

  static const size_t kMaxBufferLen = BLE_GATT_MTU_SIZE_DEFAULT - 3;
  uint8_t outBuffer_[kMaxBufferLen], inBuffer_[kMaxBufferLen];  // for outbound and inbound data (from this device)
};

#endif
