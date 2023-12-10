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
//#include "ble/BLE.h"
//#include "ble/services/HealthThermometerService.h"
#include "pretty_printer.h"
#include <MCP2515.h>
#include "slcan.h"
#include "RgbActivityLed.h"
#include "NusService.h" //deduplicate service later

#include "Bluetooth/BLE.h"
//#include <ArxTypeTraits.h>
#include "../lib/MsgPack/MsgPack.h"

#include <map>
#include <vector>
#include <cstdint>
#include <iostream>

SPI McpSpi(P0_26, P0_27, P0_7); // SI, SO, SCK
// DigitalOut McpCs(P0_27);
MCP2515 Can(McpSpi, P0_6);

BufferedSerial Uart(P1_0, NC, 115200);

Timer UsTimer;
DigitalOut LedR(P0_30), LedG(P0_5), LedB(P0_4);
RgbActivityDigitalOut StatusLed(UsTimer, LedR, LedG, LedB, false);

static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context){
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

// cs 10, so 9, si 6,sck 5
FileHandle *mbed::mbed_override_console(int){ // redirect printf to SWD UART pins
    return &Uart;
}

int main(){
    UsTimer.start();

    wait_us(10 * 1000);
    Can.setMode(CONFIGURATION);
    wait_us(10 * 1000);
    Can.baudConfig(500000);
    wait_us(10 * 1000);
    Can.setMode(NORMAL);
    Timer timer;
    timer.start();

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);

    BLEManager blemgr(ble, event_queue);
    blemgr.start();
    
    NusService bleConsole(ble);

    int currentVal = 0;

    //bleData testData{"testData", 0};

    while (1){
        event_queue.dispatch_once();
        uint8_t status = Can.readRXStatus();
        //char buf[32];
        const int data_out_size = 8;
        uint8_t data_out[data_out_size];
        bool flag = false;
        BLEManager::bleData buf;

        if ((status & 0x80) != 0 || (status & 0x40) != 0){
            flag = true;

            if((status & 0x80) != 0){
                Can.readDATA_ff_1(&buf.len_out, data_out, &buf.id);
            }
            else{ // status & 0x40
                Can.readDATA_ff_0(&buf.len_out, data_out, &buf.id);
            }

            // data
            buf.data = MsgPack::arr_t<uint8_t>(data_out, data_out + data_out_size);
            for (int i = 0; i < buf.data.size(); i++)
                buf.data[i] += 1;

            buf.state = 2; // state
            buf.id += 1; // can msg id
            buf.len_out += 1;
        }

        //

        MsgPack::Packer packer;
        //packer.serialize(3, 0, 1);
        packer.serialize(buf);

        char* rawPackedData = (char*)packer.data();

        bleConsole.write(rawPackedData);

        StatusLed.pulse(RgbActivity::kGreen);
        //}
        
        if (timer.read_ms() >= 1000){
            timer.reset();
            Can.load_ff_0(0, 0x60, NULL);
            Can.send_0();

            StatusLed.pulse(RgbActivity::kYellow);
        }

        currentVal += rawPackedData[0];

        StatusLed.update();
    }
    return 0;
}
