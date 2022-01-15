#include <events/mbed_events.h>
#include <mbed.h>
#include <ble/BLE.h>
#include <ble/services/HealthThermometerService.h>

#include "StringService.h"

// #include "RgbActivityLed.h"

// Example currently copy-pasted from https://github.com/platformio/platform-nordicnrf52/blob/master/examples/mbed-rtos-ble-thermometer/src/main.cpp
// Note Apache license.
// Will be removed once this is sufficiently refactored.


Timer UsTimer;
DigitalOut LedR(P0_10), LedG(P1_10), LedB(P1_11);
// RgbActivityDigitalOut StatusLed(UsTimer, LedR, LedG, LedB, false);

SPI SharedSpi(P0_21, P1_3, P0_19);  // mosi, miso, sck
DigitalOut AdcCs(P1_0);
DigitalOut LcdCs(P0_22);
DigitalOut LcdRs(P0_23);
DigitalOut LcdReset(P0_12);

PwmOut Speaker(P0_7);

DigitalOut MeasureSelect(P1_2);  // 0 = 1M/100 divider, 1: direct input
DigitalOut GateControl(P1_1, 1);  // power gate

DigitalIn Switch0(P0_5);  // overlaid with power switch
PwmOut DriverControl(P0_4);  // current driver setpoint
DigitalOut DriverEnable(P0_31);  // 1 = enable driver
DigitalIn Switch1(P1_4, PinMode::PullUp);  // up
DigitalIn Switch2(P0_3, PinMode::PullUp);  // down

DigitalOut InNegControl(P1_13, 1);  // 0 = GND, 1 = divider
BufferedSerial Uart(P1_15, NC, 115200);  // tx, rx


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

private:
    BLE &_ble;

    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;
};

void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}


int main() {
  Speaker.period_us(25);
  Speaker.write(0.5);

  Uart.write("BLE Multimeter\r\n", 16);
  printf("\r\n\r\n\r\n");
  printf("BLE Multimeter");
  printf("Built " __DATE__ " " __TIME__);

  uint16_t kGattServiceUuidGenericAccess = 0x1800;
  uint16_t kGattServiceUuidGenericAttribute = 0x1801;


  BLE &ble = BLE::Instance();
  ble.onEventsToProcess(schedule_ble_events);

  ThermometerDemo demo(ble, event_queue);
  demo.start();
  HealthThermometerService ThermService(ble, 0, HealthThermometerService::LOCATION_EAR);
  StringService<64> FwRevService(ble, GattCharacteristic::UUID_FIRMWARE_REVISION_STRING_CHAR, GattService::UUID_DEVICE_INFORMATION_SERVICE);

  Timer timer;
  timer.start();

  Timer audioTimer;
  audioTimer.start();

  Timer audioTimer2;
  audioTimer2.start();

  LedR = 1;
  LedG = 1;
  LedB = 1;

  while (1) {
    if (!Switch0 || !Switch1 || !Switch2) {
    // if (audioTimer2.elapsed_time().count() >= 100) {
    //   float phase = audioTimer.elapsed_time().count() / 1000000.0 * 110.0 * 2 * 3.14159;
    //   Speaker.write(0.5 + 0.5 * sin(phase));
    //   audioTimer2.reset();
    // }
    if (audioTimer2.elapsed_time().count() >= 1000) {
        if (audioTimer2.elapsed_time().count() % 9090 > 4545) {
            Speaker.write(0.25);
        } else {
            Speaker.write(0.75);
        }
        
    }
    }

    event_queue.dispatch_once();

    if (timer.read_ms() > 250) {
      timer.reset();
      LedR = !LedR;
      if (!Switch0) {
        if (LedR == 1) {
          LedB = !LedB;
        }
        ThermService.updateTemperature(LedR == 1 ? 30 : 25);
        FwRevService.writeValue(LedB == 1 ? "Ducks🦆" : "Quacks");
      }
    }

    // StatusLed.pulse(RgbActivity::kRed);
    // StatusLed.pulse(RgbActivity::kGreen);
    // StatusLed.pulse(RgbActivity::kBlue);
    // StatusLed.update();
  }
}
