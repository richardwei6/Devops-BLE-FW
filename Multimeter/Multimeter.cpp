#include <events/mbed_events.h>
#include <mbed.h>
#include <ble/BLE.h>
#include <ble/services/HealthThermometerService.h>
#include <ble/services/DeviceInformationService.h>

#include "USBSerial.h"

#include "ButtonGesture.h"
#include "RgbActivityLed.h"
#include "Mcp3561.h"
#include "StatisticalCounter.h"
#include "LongTimer.h"
#include "TickerSpeaker.h"

#include "MultimeterMeasurer.h"
#include "MultimeterDriver.h"

#include "StringService.h"
#include "NusService.h"
#include "MultimeterService.h"

#include "St7735sGraphics.h"
#include "DefaultFonts.h"
#include "Widget.h"
#include "MeasurementWidget.h"

// for itoa
#include <stdio.h>
#include <stdlib.h>

// Example currently copy-pasted from https://github.com/platformio/platform-nordicnrf52/blob/master/examples/mbed-rtos-ble-thermometer/src/main.cpp
// Note Apache license.
// Will be removed once this is sufficiently refactored.

Timer UsTimer;
DigitalOut LedR(P1_11), LedG(P1_12), LedB(P1_13);
RgbActivityDigitalOut StatusLed(UsTimer, LedR, LedG, LedB, false);

const uint32_t kAutoShutdownPeriodUs = 240 * 1000 * 1000;
Timer AutoShutdownTimer;

PwmOut SpeakerPwm(P0_14);
TickerSpeaker Speaker(SpeakerPwm, 20);
const uint32_t kSpeakerBeepFrequency = 2400;
const uint32_t kSpeakerAutoShutdownFrequency = 1200;
const float kSpeakerBeepAmplitude = 0.1;
const uint32_t kSpeakerBeepDurationUs = 100 * 1000;

DigitalOut GateControl(P0_25, 1);  // power gate

DigitalIn Switch0(P1_2);  // overlaid with power switch
ButtonGesture Switch0Gesture(Switch0);
DigitalIn Switch1(P0_27, PinMode::PullUp);  // up
ButtonGesture Switch1Gesture(Switch1);
DigitalIn Switch2(P1_10, PinMode::PullUp);  // down
ButtonGesture Switch2Gesture(Switch2);

SPI AdcSpi(P1_9, P0_8, P0_13);  // mosi, miso, sck
DigitalOut AdcCs(P0_15, 1);
Mcp3561 Adc(AdcSpi, AdcCs);
DigitalOut InNegControl(P0_17, 1);  // 0 = GND, 1 = divider
DigitalOut MeasureRange1(P0_21, 1);
DigitalOut MeasureRange0(P0_19, 1);
DigitalOut* const measureRangeControl[] = {&MeasureRange0, &MeasureRange1};
const uint16_t measureRangeRatio[] = {
  1001 / 1,  // b00, ~1:1000
  1010 / 10,  // b01, ~1:100
  1100 / 100,  // b10, ~1:10
  1  // b11, 1:1
};
uint16_t kMeasureRange1 = 0x03;  // 1:1 measurement range, b11
MultimeterMeasurer<4, 2> Meter(Adc, measureRangeRatio, measureRangeControl);

DigitalOut DriverEnable(P0_24);  // 1 = enable driver
PwmOut DriverControl(P0_23);  // current driver setpoint
// 00 = 1mA, 01 = 100uA, 10 = 10uA, 11 = 1uA
DigitalOut DriverRange0(P0_22);
DigitalOut DriverRange1(P0_20);
DigitalOut* const driverRangeControl[] = {&DriverRange0, &DriverRange1};
const uint32_t driverRangeResistance[] = {
  1000,  // b00, 1V = 1mA
  10000,  // b01, 1V = 100uA
  100000,  // b10, 1V = 10uA
  1000000  // b11, 1V = 1uA
};
const char* driverRangeString[] = {
  " 1 M ",
  "100 U",
  "10 U ",
  " 1 U "
};
uint16_t kDriverRange1Ma = 0;
MultimeterDriver<4, 2> Driver(DriverEnable, DriverControl, driverRangeResistance, driverRangeControl);

TimerTicker DebugTicker(1000 * 1000, UsTimer);
BufferedSerial SwdUart(P1_0, NC, 115200);  // tx, rx
FileHandle *mbed::mbed_override_console(int) {  // redirect printf to SWD UART pins
    return &SwdUart;
}

USBSerial UsbSerial(false, 0x1209, 0x0001, 0x0001);


//
// LCD and widgets
//
SPI LcdSpi(P0_26, NC, P0_1);  // mosi, miso, sck
DigitalOut LcdCs(P0_0, 1);
DigitalOut LcdRs(P0_29);
DigitalOut LcdReset(P1_15, 0);
St7735sGraphics<160, 80, 1, 26> Lcd(LcdSpi, LcdCs, LcdRs, LcdReset);
TimerTicker LcdUpdateTicker(100 * 1000, UsTimer);

const uint8_t kContrastActive = 255;
const uint8_t kContrastStale = 191;

const uint8_t kContrastBackground = 191;
const uint8_t kContrastInvisible = 0;

TextWidget widVersionData("BLE DMM", 0, Font5x7, kContrastActive);
TextWidget widBuildData("  " __DATE__, 0, Font5x7, kContrastBackground);
Widget* widVersionContents[] = {&widVersionData, &widBuildData};
HGridWidget<2> widVersionGrid(widVersionContents);

StaleNumericTextWidget widMeasV(0, 3, 100 * 1000, FontArial32, kContrastActive, kContrastStale, FontArial16, 1000, 3);
StaleTextWidget widMeasMode("   ", 3, 100 * 1000, FontArial16, kContrastActive, kContrastStale);
StaleTextWidget widMeasUnits("   ", 3, 100 * 1000, FontArial16, kContrastActive, kContrastStale);

MeasurementWidget widMeasurement(kContrastActive, kContrastBackground, kContrastStale, 160, 48);

Widget* measContents[] = {
    NULL, NULL, &widMeasMode,
    NULL, NULL, NULL,
    &widMeasV, NULL, &widMeasUnits
};
FixedGridWidget widMeas(measContents, 160, 48);

TextWidget widMode("     ", 5, Font5x7, kContrastStale);
LabelFrameWidget widModeFrame(&widMode, "MODE", Font3x5, kContrastBackground);
NumericTextWidget widRange(0, 5, Font5x7, kContrastStale);
LabelFrameWidget widRangeFrame(&widRange, "RANGE", Font3x5, kContrastBackground);
TextWidget widDriver("     ", 5, Font5x7, kContrastStale);
LabelFrameWidget widDriverFrame(&widDriver, "DRVR", Font3x5, kContrastBackground);

Widget* widConfigContents[] = {&widModeFrame, &widRangeFrame, &widDriverFrame};
HGridWidget<3> widConfig(widConfigContents);

NumericTextWidget widShutdown(0, 3, Font5x7, kContrastBackground);
LabelFrameWidget widShutdownFrame(&widShutdown, "SHDN", Font3x5, kContrastBackground);  // to match alignment with mode

Widget* mainContents[] = {
    NULL, NULL, NULL,
    NULL, &widMeas, NULL,
    &widConfig, NULL, &widShutdownFrame
};
FixedGridWidget widMain(mainContents, 160, 80);


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

        ble::own_address_type_t addr_type;
        ble::address_t address;
        _ble.gap().getAddress(addr_type, address);
        printf("Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
            address.data()[5], address.data()[4], address.data()[3], address.data()[2], address.data()[1], address.data()[0]);

        start_advertising();
    }

    void start_advertising() {
        /* Create advertising parameters and payload */

        ble::AdvertisingParameters adv_parameters(
            ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
            ble::adv_interval_t(ble::millisecond_t(1000))
        );

        const UUID deviceInformationService = GattService::UUID_DEVICE_INFORMATION_SERVICE;
        const UUID uartService = NusService::kServiceUuid;
        const UUID services[] = {deviceInformationService, uartService};

        _adv_data_builder.setFlags();
        // _adv_data_builder.setLocalServiceList(mbed::make_Span(services, 2));  // somehow breaks names
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
StatisticalCounter<int32_t, int64_t> AdcStats;
StatisticalCounter<int32_t, int64_t> VoltageStats;
enum kMultimeterMode {
  kVoltage = 0,
  kResistance = 1,
  kDiode = 2,
  kContinuity = 3
};


int main() {
  // Set SWO pin into GPIO mode
  NRF_CLOCK->TRACECONFIG = 0;

  printf("\r\n\r\n\r\n");
  printf("BLE Multimeter\n");
  printf("Built " __DATE__ " " __TIME__ "\n");

  BLE &ble = BLE::Instance();
  ble.onEventsToProcess(schedule_ble_events);

  ThermometerDemo demo(ble, event_queue);
  demo.start();

  VoltmeterService bleVoltmeter(ble, 0x183B);
  DeviceInformationService bleDeviceInfo(ble, "Ducky", "Multimeter", "0001",
                                      "rv1", __DATE__ " " __TIME__, "NA");
  NusService bleConsole(ble);  

  UsTimer.start();
  AutoShutdownTimer.start();
  uint32_t lastShutdownRemainingMs = UINT32_MAX;

  LedR = 1;
  LedG = 1;
  LedB = 1;

  LcdSpi.format(8, 0);
  LcdSpi.frequency(15 * 1000 * 1000);  // 66ns max SCK cycle, for write ops
  Lcd.init();

  AdcSpi.format(8, 0);
  AdcSpi.frequency(20 * 1000 * 1000);
  Adc.fullReset();
  Adc.init(Mcp3561::kOsr::k98304);
  uint8_t statusCode = Adc.startConversion();

  Speaker.tone(kSpeakerBeepFrequency, kSpeakerBeepAmplitude, kSpeakerBeepDurationUs);

  printf("Status=%02x, ADC Configs 0=%02x, 1=%02x, 2=%02x, 3=%02x, MUX=%02x\n", statusCode,
    Adc.readReg8(Mcp3561::kRegister::CONFIG0), Adc.readReg8(Mcp3561::kRegister::CONFIG1),
    Adc.readReg8(Mcp3561::kRegister::CONFIG2), Adc.readReg8(Mcp3561::kRegister::CONFIG3),
    Adc.readReg8(Mcp3561::kRegister::MUX));

  bool sw0Released = false;
  uint16_t rangeDivide = 0;

  kMultimeterMode mode = kMultimeterMode::kVoltage;
  kMultimeterMode lastMode = (kMultimeterMode)(mode - 1);  // run mode entry routine on first cycle
  bool continuityTonePlaying = false;

  while (1) {
    if (UsbSerial.connected()) {
      StatusLed.setIdle(RgbActivity::kGreen);
    } else if (UsbSerial.configured()) {
      StatusLed.setIdle(RgbActivity::kYellow);
    } else {
      UsbSerial.connect();
      StatusLed.setIdle(RgbActivity::kOff);
    }

    if (Switch0 == 1) {  // make sure to not detect anything related to the power on button
      sw0Released = true;
    }
    if (sw0Released) {
      switch (Switch0Gesture.update()) {
        case ButtonGesture::Gesture::kClickRelease:
          mode = (kMultimeterMode)((mode + 1) % 4);
          AutoShutdownTimer.reset();
          Speaker.tone(kSpeakerBeepFrequency, kSpeakerBeepAmplitude, kSpeakerBeepDurationUs);
          break;
        case ButtonGesture::Gesture::kHoldTransition:  // long press to shut off
          GateControl = 0;
          break;
        default: break;
      }
    }

    switch (Switch1Gesture.update()) {
      default: break;
    }

    switch (Switch2Gesture.update()) {
      default: break;
    }

    int32_t voltage, adcValue;
    if (Meter.readVoltageMv(&voltage, &adcValue, &rangeDivide)) {
      switch (mode) {
        case kMultimeterMode::kVoltage:
          if (lastMode != mode) {
            Adc.init(Mcp3561::kOsr::k98304);  // high precision mode
            Meter.setRange(kMeasureRange1);  // starting range
            InNegControl = 1;
            Driver.enable(false);

            widMode.setValue(" VLT ");
            widDriver.setValue(" OFF ");
            widMeasMode.setValue(" DC");
            widMeasUnits.setValue("  V");
          }
          Meter.autoRange(adcValue);
          widMeasV.setValue(voltage);
          widRange.setValue(rangeDivide);

          if (rangeDivide > 1) {  // range beyond 1:1 indicates liveness (1:1 range floats and is unreliable)
            AutoShutdownTimer.reset();
          }
          break;
        case kMultimeterMode::kResistance:
          if (lastMode != mode) {
            Adc.init(Mcp3561::kOsr::k98304);  // high precision mode
            Meter.setRange(kMeasureRange1);
            InNegControl = 0;
            Driver.setRange(kDriverRange1Ma);  // starting range
            Driver.enable(true);

            widMode.setValue(" RES ");
            widRange.setValue(1);
            widDriver.setValue(driverRangeString[Driver.getRange()]);
            widMeasMode.setValue("   ");
            widMeasUnits.setValue("  R");
          }
          widMeasV.setValue((int64_t)voltage * 1000000 / Driver.getCurrentUa());
          widDriver.setValue(driverRangeString[Driver.getRange()]);

          if (Driver.getRange() < sizeof(driverRangeResistance) / sizeof(driverRangeResistance[0])) {  // range beyond max (open) indicates liveness
            AutoShutdownTimer.reset();
          }
          Driver.autoRangeResistance(voltage);
          break;
        case kMultimeterMode::kDiode:
          if (lastMode != mode) {
            Adc.init(Mcp3561::kOsr::k98304);  // high precision mode
            InNegControl = 0;
            Driver.setRange(kDriverRange1Ma);
            Driver.enable(true);

            widMode.setValue(" DIO ");
            widDriver.setValue(driverRangeString[Driver.getRange()]);
            widMeasMode.setValue(" Vf");
            widMeasUnits.setValue("  V");
          }
          Meter.autoRange(adcValue);  // diodes may exceed 1:1 range
          widMeasV.setValue(voltage);
          widRange.setValue(rangeDivide);

          if (voltage < 1800) { // arbitrary threshold for "something is there"
            AutoShutdownTimer.reset();
          }
          break;
        case kMultimeterMode::kContinuity:
          if (lastMode != mode) {
            Adc.init(Mcp3561::kOsr::k2048);  // fast mode
            Meter.setRange(kMeasureRange1);
            InNegControl = 0;
            Driver.setRange(kDriverRange1Ma);
            Driver.enable(true);
            continuityTonePlaying = false;

            widMode.setValue(" CON ");
            widRange.setValue(1);
            widDriver.setValue(driverRangeString[Driver.getRange()]);
            widMeasMode.setValue("   ");
            widMeasUnits.setValue("  R");
          }
          widMeasV.setValue(voltage);
          if (voltage < 100) {
            if (!continuityTonePlaying) {
              Speaker.tone(kSpeakerBeepFrequency, kSpeakerBeepAmplitude);
              continuityTonePlaying = true;
              AutoShutdownTimer.reset();  // continuity indicates liveness
            }
          } else {
            if (continuityTonePlaying) {
              Speaker.tone(0, 0);
              continuityTonePlaying = false;
            }
          }
          break;
        default:
          widMode.setValue(" UNK ");
          InNegControl = 1;
          Driver.enable(false);
          break;
      }
      lastMode = mode;
      Adc.startConversion();

      AdcStats.addSample(adcValue);
      VoltageStats.addSample(voltage);
    }

    if (DriverEnable == 1) {
      StatusLed.pulse(RgbActivity::kRed);
    } 

    if (DebugTicker.checkExpired()) {
      auto adcStats = AdcStats.read();
      auto voltageStats = VoltageStats.read();
      AdcStats.reset();
      VoltageStats.reset();

      bleVoltmeter.writeVoltage(voltageStats.avg);

      // debugging stuff below
      printf("NC=%i, Div=%u, ADC(%u) = %li - %li - %li (%lu)    V(%u) = %li - %li - %li (%li)\n", 
          InNegControl.read(), rangeDivide,
          adcStats.numSamples, adcStats.min, adcStats.avg, adcStats.max, 
          adcStats.max-adcStats.min,
          voltageStats.numSamples, voltageStats.min, voltageStats.avg, voltageStats.max, 
          voltageStats.max-voltageStats.min);

      if (UsbSerial.connected()) {
        UsbSerial.printf("\n");
      }

      char voltsStr[128];
      itoa(voltageStats.avg, voltsStr, 10);
      strcat(voltsStr, " mV\n");
      bleConsole.write(voltsStr);
    }

    event_queue.dispatch_once();

    int32_t shutdownRemainingMs = int32_t(kAutoShutdownPeriodUs / 1000) - AutoShutdownTimer.read_ms();
    if (shutdownRemainingMs >= 0) {
      if (shutdownRemainingMs < 2000) {
        if ((lastShutdownRemainingMs % 500) < (shutdownRemainingMs % 500)) {
          Speaker.tone(kSpeakerAutoShutdownFrequency, kSpeakerBeepAmplitude, kSpeakerBeepDurationUs);
        }
      } else if (shutdownRemainingMs <= 10000) {
        if ((lastShutdownRemainingMs % 2000) < (shutdownRemainingMs % 2000)) {
          Speaker.tone(kSpeakerAutoShutdownFrequency, kSpeakerBeepAmplitude, kSpeakerBeepDurationUs);
        }
      }
      widShutdown.setValue(shutdownRemainingMs / 1000);
    } else {
      widShutdown.setValue(0);
      GateControl = 0;
    }
    lastShutdownRemainingMs = shutdownRemainingMs;
    
    // TODO: this takes a long time, this may mess with continuity mode
    if (LcdUpdateTicker.checkExpired()) {
      Lcd.clear();
      widMain.layout();
      widMain.draw(Lcd, 0, 0);
      Lcd.update();
    }

    StatusLed.update();
  }
}
