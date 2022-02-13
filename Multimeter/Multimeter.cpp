#include <events/mbed_events.h>
#include <mbed.h>
#include <ble/BLE.h>
#include <ble/services/HealthThermometerService.h>
#include <ble/services/DeviceInformationService.h>

#include "USBSerial.h"

#include "ButtonGesture.h"
#include "RgbActivityLed.h"
#include "Mcp3201.h"
#include "StatisticalCounter.h"

#include "MultimeterMeasurer.h"
#include "MultimeterDriver.h"

#include "StringService.h"

// Example currently copy-pasted from https://github.com/platformio/platform-nordicnrf52/blob/master/examples/mbed-rtos-ble-thermometer/src/main.cpp
// Note Apache license.
// Will be removed once this is sufficiently refactored.

Timer UsTimer;
DigitalOut LedR(P0_10), LedG(P1_10), LedB(P1_11);
RgbActivityDigitalOut StatusLed(UsTimer, LedR, LedG, LedB, false);

// SPI SharedSpi(P0_21, P1_3, P0_19);  // mosi, miso, sck
SPI SharedSpi(P0_21, P1_3, P1_0);  // mosi, miso, sck - SCK and CS swapped for MCP3550
DigitalOut LcdCs(P0_22, 1);
DigitalOut LcdRs(P0_23);
DigitalOut LcdReset(P0_12, 0);

PwmOut Speaker(P0_7);

DigitalOut GateControl(P1_1, 1);  // power gate

DigitalIn Switch0(P0_5);  // overlaid with power switch
ButtonGesture Switch0Gesture(Switch0);
DigitalIn Switch1(P1_4, PinMode::PullUp);  // up
ButtonGesture Switch1Gesture(Switch1);
DigitalIn Switch2(P0_3, PinMode::PullUp);  // down
ButtonGesture Switch2Gesture(Switch2);

BufferedSerial SwdUart(P1_15, NC, 115200);  // tx, rx

// DigitalOut AdcCs(P1_0, 1);  // SCK and CS swapped for MCP3550
DigitalOut AdcCs(P0_19, 1);
DigitalIn AdcSo(P1_3, PinMode::PullUp);
Mcp3550 Adc(SharedSpi, AdcCs, AdcSo);
DigitalOut MeasureSelect(P1_2);  // 0 = 1M/100 divider, 1: direct input
DigitalOut InNegControl(P1_13, 1);  // 0 = GND, 1 = divider
MultimeterMeasurer Meter(Adc, MeasureSelect, InNegControl);

DigitalOut DriverEnable(P0_31);  // 1 = enable driver
PwmOut DriverControl(P0_4);  // current driver setpoint
MultimeterDriver Driver(DriverEnable, DriverControl);

FileHandle *mbed::mbed_override_console(int) {  // redirect printf to SWD UART pins
    return &SwdUart;
}


USBSerial UsbSerial(false, 0x1209, 0x0001, 0x0001);


// BLE comms
const static char DEVICE_NAME[] = "DuckyMultimeter";
const size_t kEventQueueSize = 16;
static events::EventQueue event_queue(kEventQueueSize * EVENTS_EVENT_SIZE);

class ThermometerDemo : ble::Gap::EventHandler {
public:
    ThermometerDemo(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _adv_data_builder(_adv_buffer) { }

    void start() {
        _ble.gap().setEventHandler(this);

        _ble.init(this, &ThermometerDemo::on_init_complete);
    }

private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
        if (params->error != BLE_ERROR_NONE) {
            // print_error(params->error, "Ble initialization failed.");
            return;
        }

        start_advertising();
    }

    void start_advertising() {
        /* Create advertising parameters and payload */

        ble::AdvertisingParameters adv_parameters(
            ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
            ble::adv_interval_t(ble::millisecond_t(1000))
        );

        _adv_data_builder.setFlags();
        _adv_data_builder.setAppearance(ble::adv_data_appearance_t::DIGITAL_PEN);
        _adv_data_builder.setName(DEVICE_NAME);

        /* Setup advertising */

        ble_error_t error = _ble.gap().setAdvertisingParameters(
            ble::LEGACY_ADVERTISING_HANDLE,
            adv_parameters
        );

        if (error) {
            // print_error(error, "_ble.gap().setAdvertisingParameters() failed");
            return;
        }

        error = _ble.gap().setAdvertisingPayload(
            ble::LEGACY_ADVERTISING_HANDLE,
            _adv_data_builder.getAdvertisingData()
        );

        if (error) {
            // print_error(error, "_ble.gap().setAdvertisingPayload() failed");
            return;
        }

        /* Start advertising */

        error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

        if (error) {
            // print_error(error, "_ble.gap().startAdvertising() failed");
            return;
        }
    }

    virtual void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
         _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
    }

private:
    BLE &_ble;

    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;
};

void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}


// Application-specific temporary
StatisticalCounter<uint32_t, uint64_t> AdcStats;
StatisticalCounter<int32_t, int64_t> VoltageStats;
Timer ConvTimer;


int main() {
  // Set SWO pin into GPIO mode, since it's used for the ADC CS
  NRF_CLOCK->TRACECONFIG = 0;

  Speaker.period_us(25);
  Speaker.write(0.5);

  printf("\r\n\r\n\r\n");
  printf("BLE Multimeter\n");
  printf("Built " __DATE__ " " __TIME__ "\n");

  uint16_t kGattServiceUuidGenericAccess = 0x1800;
  uint16_t kGattServiceUuidGenericAttribute = 0x1801;


  BLE &ble = BLE::Instance();
  ble.onEventsToProcess(schedule_ble_events);

  ThermometerDemo demo(ble, event_queue);
  demo.start();
  HealthThermometerService ThermService(ble, 0, HealthThermometerService::LOCATION_EAR);
  DeviceInformationService DeviceInfo(ble, "Ducky", "Multimeter", "0001",
                                      "rv1", __DATE__ " " __TIME__, "NA");
  StringService<64> FwRevService(ble, GattCharacteristic::UUID_FIRMWARE_REVISION_STRING_CHAR, kGattServiceUuidGenericAccess);

  UsTimer.start();

  Timer timer;
  timer.start();

  Timer audioTimer;
  audioTimer.start();

  Timer audioTimer2;
  audioTimer2.start();

  ConvTimer.start();

  LedR = 1;
  LedG = 1;
  LedB = 1;

  // Driver.enable();
  Driver.setCurrent(2000);

  while (1) {
    if (!Switch0 || !Switch1 || !Switch2) {
    // if (audioTimer2.elapsed_time().count() >= 100) {
    //   float phase = audioTimer.elapsed_time().count() / 1000000.0 * 110.0 * 2 * 3.14159;
    //   Speaker.write(0.5 + 0.5 * sin(phase));
    //   audioTimer2.reset();
    // }
    // if (audioTimer2.elapsed_time().count() >= 1000) {
    //     if (audioTimer2.elapsed_time().count() % 9090 > 4545) {
    //         Speaker.write(0.25);
    //     } else {
    //         Speaker.write(0.75);
    //     }
    // }
    }

    event_queue.dispatch_once();

    if (UsbSerial.connected()) {
      StatusLed.setIdle(RgbActivity::kGreen);
    } else if (UsbSerial.configured()) {
      StatusLed.setIdle(RgbActivity::kYellow);
    } else {
      UsbSerial.connect();
      StatusLed.setIdle(RgbActivity::kOff);
    }

    switch (Switch0Gesture.update()) {
      case ButtonGesture::Gesture::kClickRelease:  // test code
        ThermService.updateTemperature(LedR == 1 ? 30 : 25);
        // FwRevService.writeValue(LedB == 1 ? "Ducks🦆" : "Quacks");
        break;
      case ButtonGesture::Gesture::kHoldTransition:  // long press to shut off
        GateControl = 0;
        break;
      default: break;
    }

    switch (Switch1Gesture.update()) {
      default: break;
    }

    switch (Switch2Gesture.update()) {
      case ButtonGesture::Gesture::kClickRelease:
        MeasureSelect = !MeasureSelect;
        break;
      default: break;
    }
      
    uint32_t adcValue;
    int32_t voltage;
    if (Meter.readVoltageMv(&voltage, &adcValue)) {
      StatusLed.pulse(RgbActivity::kCyan);
      AdcStats.addSample(adcValue);
      VoltageStats.addSample(voltage);
      // printf("% 3lims    ADC=%li lsb    V=%li mV\n", 
      //   ConvTimer.read_ms(), adcValue, voltage);
      // ConvTimer.reset();
    }

    if (timer.read_ms() >= 2500) {
      timer.reset();
      auto adcStats = AdcStats.read();
      auto voltageStats = VoltageStats.read();
      AdcStats.reset();
      VoltageStats.reset();

      printf("MS=%i, NC=%i, ADC(%u) = %lu - %lu - %lu (%lu)    V(%u) = %li - %li - %li (%li)\n", 
          MeasureSelect.read(), InNegControl.read(),
          adcStats.numSamples, adcStats.min, adcStats.avg, adcStats.max, 
          adcStats.max-adcStats.min,
          voltageStats.numSamples, voltageStats.min, voltageStats.avg, voltageStats.max, 
          voltageStats.max-voltageStats.min);

      if (UsbSerial.connected()) {
        UsbSerial.printf("\n");
      }
    }

    StatusLed.update();
  }
}
