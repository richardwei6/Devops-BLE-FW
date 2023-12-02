#include "../ble/BLE.h"
//#include "../ble/services/HealthThermometerService.h"
#include <events/mbed_events.h>
#include "../../lib/MsgPack/MsgPack.h"


const static char DEVICE_NAME[] = "BLETelemetry1";

class BLEManager : ble::Gap::EventHandler
{
public:
    BLEManager(BLE &ble, events::EventQueue &event_queue) : _ble(ble),
                                                                 _event_queue(event_queue),
                                                                 //_sensor_event_id(0),
                                                                 //_thermometer_uuid(GattService::UUID_HEALTH_THERMOMETER_SERVICE),
                                                                 //_current_temperature(39.6f),
                                                                 //_thermometer_service(NULL),
                                                                 _adv_data_builder(_adv_buffer) {}

    void start();

    struct bleData{
        uint16_t id;
        MsgPack::arr_t<uint8_t> data;
        uint8_t len_out;
        MSGPACK_DEFINE(id, data, len_out);
    };

private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params);

    void start_advertising();
  

    /*void update_sensor_value()
    {
        //_current_temperature = (_current_temperature + 0.1f > 43.0f) ? 39.6f : _current_temperature + 0.1f;
        //_current_temperature = 0.1f;
        //_thermometer_service->updateTemperature(_current_temperature);
    }*/

private:
    /* Event handler */

    virtual void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &); 

    virtual void onConnectionComplete(const ble::ConnectionCompleteEvent &event);


private:
    BLE &_ble;
    events::EventQueue &_event_queue;

    int _sensor_event_id;

    //UUID _thermometer_uuid;

    //float _current_temperature;
    //HealthThermometerService *_thermometer_service;

    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;
};

