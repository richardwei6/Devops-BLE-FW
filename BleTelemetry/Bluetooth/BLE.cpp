#include "BLE.h"
#include "../pretty_printer.h"

void BLEManager::start(){
     _ble.gap().setEventHandler(this);
     _ble.init(this, &BLEManager::on_init_complete);
}

//

void BLEManager::on_init_complete(BLE::InitializationCompleteCallbackContext *params){
    if (params->error != BLE_ERROR_NONE)
    {
        print_error(params->error, "Ble initialization failed.");
        return;
    }

    print_mac_address();

    /* Setup primary service. */
    //_thermometer_service = new HealthThermometerService(_ble, _current_temperature, HealthThermometerService::LOCATION_EAR);

    start_advertising();
}

void BLEManager::start_advertising()  {
    /* Create advertising parameters and payload */

    ble::AdvertisingParameters adv_parameters(
        ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
        ble::adv_interval_t(ble::millisecond_t(1000)));
    //const UUID uartService = NusService::kServiceUuid;

    _adv_data_builder.setFlags();
    //_adv_data_builder.setLocalServiceList(mbed::make_Span(&_thermometer_uuid, 1));
    //_adv_data_builder.setAppearance(ble::adv_data_appearance_t::THERMOMETER_EAR);
    //ble::adv_data_appearance_t
    _adv_data_builder.setAppearance(ble::adv_data_appearance_t::UNKNOWN);
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

//

void BLEManager::onDisconnectionComplete(const ble::DisconnectionCompleteEvent &){
    _event_queue.cancel(_sensor_event_id);
    _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
}

void BLEManager::onConnectionComplete(const ble::ConnectionCompleteEvent &event){
    if (event.getStatus() == BLE_ERROR_NONE){
        //_sensor_event_id = _event_queue.call_every(1000, this, &BLEManager::update_sensor_value);
    }
}

//