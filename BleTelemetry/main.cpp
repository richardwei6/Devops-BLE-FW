/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// schematic for nrf https://cdn-learn.adafruit.com/assets/assets/000/068/545/original/circuitpython_nRF52840_Schematic_REV-D.png
#include <events/mbed_events.h>
#include "mbed.h"
#include "ble/BLE.h"
#include "ble/services/HealthThermometerService.h"
#include "pretty_printer.h"
#include <MCP2515.h>
#include "slcan.h"
#include "NusService.h" //deduplicate service later
#include "RgbActivityLed.h"

SPI McpSpi(P0_26, P0_27, P0_7); // SI, SO, SCK
// DigitalOut McpCs(P0_27);
MCP2515 Can(McpSpi, P0_6);

BufferedSerial Uart(P1_0, NC, 115200);

Timer UsTimer;
DigitalOut LedR(P0_30), LedG(P0_5), LedB(P0_4);
RgbActivityDigitalOut StatusLed(UsTimer, LedR, LedG, LedB, false);

// cs 10, so 9, si 6,sck 5
FileHandle *mbed::mbed_override_console(int)
{ // redirect printf to SWD UART pins
    return &Uart;
}

const static char DEVICE_NAME[] = "BLETelemetry1";

static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);

class ThermometerDemo : ble::Gap::EventHandler
{
public:
    ThermometerDemo(BLE &ble, events::EventQueue &event_queue) : _ble(ble),
                                                                 _event_queue(event_queue),
                                                                 _sensor_event_id(0),
                                                                 _thermometer_uuid(GattService::UUID_HEALTH_THERMOMETER_SERVICE),
                                                                 _current_temperature(39.6f),
                                                                 _thermometer_service(NULL),
                                                                 _adv_data_builder(_adv_buffer) {}

    void start()
    {
        _ble.gap().setEventHandler(this);

        _ble.init(this, &ThermometerDemo::on_init_complete);
    }

private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params)
    {
        if (params->error != BLE_ERROR_NONE)
        {
            print_error(params->error, "Ble initialization failed.");
            return;
        }

        print_mac_address();

        /* Setup primary service. */
        _thermometer_service = new HealthThermometerService(_ble, _current_temperature, HealthThermometerService::LOCATION_EAR);

        start_advertising();
    }

    void start_advertising()
    {
        /* Create advertising parameters and payload */

        ble::AdvertisingParameters adv_parameters(
            ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
            ble::adv_interval_t(ble::millisecond_t(1000)));
        const UUID uartService = NusService::kServiceUuid;

        _adv_data_builder.setFlags();
        _adv_data_builder.setLocalServiceList(mbed::make_Span(&_thermometer_uuid, 1));
        _adv_data_builder.setAppearance(ble::adv_data_appearance_t::THERMOMETER_EAR);
        _adv_data_builder.setName(DEVICE_NAME);

        /* Setup advertising */

        ble_error_t error = _ble.gap().setAdvertisingParameters(
            ble::LEGACY_ADVERTISING_HANDLE,
            adv_parameters);

        if (error)
        {
            print_error(error, "_ble.gap().setAdvertisingParameters() failed");
            return;
        }

        error = _ble.gap().setAdvertisingPayload(
            ble::LEGACY_ADVERTISING_HANDLE,
            _adv_data_builder.getAdvertisingData());

        if (error)
        {
            print_error(error, "_ble.gap().setAdvertisingPayload() failed");
            return;
        }

        /* Start advertising */

        error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

        if (error)
        {
            print_error(error, "_ble.gap().startAdvertising() failed");
            return;
        }
    }

    void update_sensor_value()
    {
        //_current_temperature = (_current_temperature + 0.1f > 43.0f) ? 39.6f : _current_temperature + 0.1f;
        _current_temperature = 0.1f;
        _thermometer_service->updateTemperature(_current_temperature);
    }

private:
    /* Event handler */

    virtual void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &)
    {
        _event_queue.cancel(_sensor_event_id);
        _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
    }

    virtual void onConnectionComplete(const ble::ConnectionCompleteEvent &event)
    {
        if (event.getStatus() == BLE_ERROR_NONE)
        {
            _sensor_event_id = _event_queue.call_every(1000, this, &ThermometerDemo::update_sensor_value);
        }
    }

private:
    BLE &_ble;
    events::EventQueue &_event_queue;

    int _sensor_event_id;

    UUID _thermometer_uuid;

    float _current_temperature;
    HealthThermometerService *_thermometer_service;

    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;
};

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context)
{
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

int main()
{
    UsTimer.start();

    wait_us(10 * 1000);
    Can.setMode(CONFIGURATION);
    wait_us(10 * 1000);
    Can.baudConfig(1000);
    wait_us(10 * 1000);
    Can.setMode(NORMAL);
    Timer timer;
    timer.start();

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);
    ThermometerDemo demo(ble, event_queue);
    demo.start();
    NusService bleConsole(ble);

    while (1)
    {
        event_queue.dispatch_once();
        uint8_t status = Can.readRXStatus();
        uint8_t len_out;
        uint8_t data_out[8];
        uint16_t id;
        char buf[32];
        bool flag = false;

        /*if((status & 0x80) != 0){
            flag = true;
            Can.readDATA_ff_1(&len_out, data_out, &id);
        }
        else if((status & 0x40) != 0){
            flag= true;
            Can.readDATA_ff_0(&len_out, data_out, &id);
        }*/
        if (!flag){
            flag = true;

            id = 1;
            len_out = 1;
            data_out[0] = 'c';

            //std::string testStr = "This is a test string";
            //strcpy(buf, testStr.c_str());
        }

        if (flag)
        {
            CANMessage msg(id, data_out, len_out);
            size_t len = SLCANBase::formatCANMessage(msg, buf, sizeof(buf));
            buf[len] = '\0';
            bleConsole.write(buf);

            StatusLed.pulse(RgbActivity::kGreen);
        }
        
        if (timer.read_ms() >= 1000)
        {
            timer.reset();
            Can.load_ff_0(0, 0x60, NULL);
            Can.send_0();

            StatusLed.pulse(RgbActivity::kYellow);
        }

        StatusLed.update();
    }
    return 0;
}
