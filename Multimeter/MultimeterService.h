#ifndef __MULTIMETER_SERVICE_H__
#define __MULTIMETER_SERVICE_H__

#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/GattServer.h"


/**
* Implements a BLE service (with user-defined service UUID) that contains the standard voltage characteristic.
*/
class MultimeterService {
public:
  MultimeterService(BLE &ble) : ble_(ble),
      voltageCharacteristic_(kUuidReading, &voltage_, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),
      modeCharacteristic_(kUuidMode, &mode_, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),
      adcCharacteristic_(kUuidAdc, &adc_, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),
      resistanceCharacteristic_(kUuidResistance, &resistance_, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY)
        {
    GattCharacteristic *charTable[] = {
      &voltageCharacteristic_, &modeCharacteristic_,
      &adcCharacteristic_, &resistanceCharacteristic_, 
      };
    GattService service(kUuidService, charTable, sizeof(charTable) / sizeof(charTable[0]));
    ble.gattServer().addService(service);
  }

  void writeVoltage(int32_t voltageMv) {
    ble_.gattServer().write(voltageCharacteristic_.getValueHandle(), (uint8_t*)&voltageMv, sizeof(voltageMv));
  }

  void writeMode(uint8_t mode) {
    ble_.gattServer().write(modeCharacteristic_.getValueHandle(), (uint8_t*)&mode, sizeof(mode));
  }

  void writeAdc(uint32_t adc) {
    ble_.gattServer().write(adcCharacteristic_.getValueHandle(), (uint8_t*)&adc, sizeof(adc));
  }

  void writeResistance(uint32_t resistanceMOhm) {
    ble_.gattServer().write(resistanceCharacteristic_.getValueHandle(), (uint8_t*)&resistanceMOhm, sizeof(resistanceMOhm));
  }


  static constexpr uint8_t kUuidService[UUID::LENGTH_OF_LONG_UUID] = {
    0x0b, 0xde, 0x0a, 0x01,
    0xcd, 0xb0,
    0x46, 0xbd, 
    0xa3, 0xa2,
    0x85, 0xb7, 0xe1, 0x63, 0x29, 0xd1
  };
  // UUID offset applied to the 4th byte
  static constexpr uint8_t kOffsetReading = 1;
  static constexpr uint8_t kOffsetMode = 2;
  static constexpr uint8_t kOffsetAdc = 3;
  static constexpr uint8_t kOffsetResistance = 4;

  // TODO deduplicate this mess
  static constexpr uint8_t kUuidReading[UUID::LENGTH_OF_LONG_UUID] = {
    0x0b, 0xde, 0x0a, 0x01 + kOffsetReading,
    0xcd, 0xb0,  0x46, 0xbd,  0xa3, 0xa2,
    0x85, 0xb7, 0xe1, 0x63, 0x29, 0xd1
  };
  static constexpr uint8_t kUuidMode[UUID::LENGTH_OF_LONG_UUID] = {
    0x0b, 0xde, 0x0a, 0x01 + kOffsetMode,
    0xcd, 0xb0,  0x46, 0xbd,  0xa3, 0xa2,
    0x85, 0xb7, 0xe1, 0x63, 0x29, 0xd1
  };
  static constexpr uint8_t kUuidAdc[UUID::LENGTH_OF_LONG_UUID] = {
    0x0b, 0xde, 0x0a, 0x01 + kOffsetAdc,
    0xcd, 0xb0,  0x46, 0xbd,  0xa3, 0xa2,
    0x85, 0xb7, 0xe1, 0x63, 0x29, 0xd1
  };
  static constexpr uint8_t kUuidResistance[UUID::LENGTH_OF_LONG_UUID] = {
    0x0b, 0xde, 0x0a, 0x01 + kOffsetResistance,
    0xcd, 0xb0,  0x46, 0xbd,  0xa3, 0xa2,
    0x85, 0xb7, 0xe1, 0x63, 0x29, 0xd1
  };

protected:
  BLE &ble_;
  ReadOnlyGattCharacteristic<int32_t> voltageCharacteristic_;  // 4-byte signed, mV reading
  ReadOnlyGattCharacteristic<uint8_t> modeCharacteristic_;  // 1 byte R/W, corresponds to kMultimeterMode
  ReadOnlyGattCharacteristic<int32_t> adcCharacteristic_;  // 4-byte signed, raw ADC counts
  ReadOnlyGattCharacteristic<uint32_t> resistanceCharacteristic_;  // 4-byte unsigned, derived mOhms resistance, only updated in resistance or continuity
  int32_t voltage_;
  uint8_t mode_;
  int32_t adc_;
  uint32_t resistance_;
};

#endif
