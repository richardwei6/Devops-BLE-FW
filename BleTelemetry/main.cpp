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
//#include "pretty_printer.h"
#include <MCP2515.h>
#include "slcan.h"
#include "NusService.h" //deduplicate service later
#include "RgbActivityLed.h"

#include "Bluetooth/BLE.h"

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

    int flip = 0;

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
            data_out[0] = (char)('T' + flip);

            flip = 1 - flip;

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
