#ifndef __MULTIMETER_SERVICE_H__
#define __MULTIMETER_SERVICE_H__

#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/GattServer.h"

/**
* Implements a BLE service (with user-defined service UUID) that contains the standard voltage characteristic.
*/
class VoltmeterService {
public:
  VoltmeterService(BLE &ble, UUID serviceUuid) : ble_(ble),
      voltageCharacteristic_(kCharacteristicVoltage, &voltage_, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE) {
    GattCharacteristic *charTable[] = {&voltageCharacteristic_, };
    GattService service(serviceUuid, charTable, sizeof(charTable) / sizeof(charTable[0]));
    ble.gattServer().addService(service);
  }

  void writeVoltage(int32_t voltageMv) {
    voltage_ = voltageMv * 64 / 1000;  // scale to LSB = 1/64V
    ble_.gattServer().write(voltageCharacteristic_.getValueHandle(), (uint8_t*)&voltage_, sizeof(voltage_));
  }

  uint16_t kCharacteristicVoltage = 0x2b18;

protected:
  BLE &ble_;
  ReadOnlyGattCharacteristic<uint16_t> voltageCharacteristic_;
  uint16_t voltage_;
};

#endif
