#include <events/mbed_events.h>
#include <mbed.h>
#include <ble/BLE.h>
#include <ble/services/HealthThermometerService.h>

// #include "RgbActivityLed.h"

// Example currently copy-pasted from https://github.com/platformio/platform-nordicnrf52/blob/master/examples/mbed-rtos-ble-thermometer/src/main.cpp
// Note Apache license.
// Will be removed once this is sufficiently refactored.


Timer UsTimer;
DigitalOut LedR(P0_10), LedG(P1_10), LedB(P1_11);
// RgbActivityDigitalOut StatusLed(UsTimer, LedR, LedG, LedB, false);

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
        UUID thermometerUuid = GattService::UUID_HEALTH_THERMOMETER_SERVICE;
        _adv_data_builder.setLocalServiceList(mbed::make_Span(&thermometerUuid, 1));
        _adv_data_builder.setAppearance(ble::adv_data_appearance_t::THERMOMETER_EAR);
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
    /* Event handler */
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


int main() {
  BLE &ble = BLE::Instance();
  ble.onEventsToProcess(schedule_ble_events);

  ThermometerDemo demo(ble, event_queue);
  demo.start();
  HealthThermometerService ThermService(ble, 0, HealthThermometerService::LOCATION_EAR);

  Timer timer;
  timer.start();

  LedR = 1;
  LedG = 1;
  LedB = 1;

  while (1) {
    event_queue.dispatch_once();

    if (timer.read_ms() > 250) {
      timer.reset();
      LedR = !LedR;
      ThermService.updateTemperature(LedR == 1 ? 30 : 25);
    }

    // StatusLed.pulse(RgbActivity::kRed);
    // StatusLed.pulse(RgbActivity::kGreen);
    // StatusLed.pulse(RgbActivity::kBlue);
    // StatusLed.update();
  }
}
