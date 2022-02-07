#ifndef __MULTIMETER_MEASURER_H__
#define __MULTIMETER_MEASURER_H__
#include <mbed.h>
#include "Mcp3201.h"


/**
 * Volts measurement stage for the multimeter.
 * Optional auto-ranging.
 */
class MultimeterMeasurer {
public:
  enum Range {
    kRange1,
    kRange100,
  };

  MultimeterMeasurer(Mcp3201& adc, DigitalOut& measureSelect, DigitalOut& referenceSelect) :
      adc_(adc), measureSelect_(measureSelect), referenceSelect_(referenceSelect) {
  }

  void setRange(Range range) {
    switch (range) {
      case kRange1:  measureSelect_ = 1;  break;
      case kRange100:  measureSelect_ = 0;  break;
      default: break;
    }
  }

  // 
  int32_t readVoltageMv(uint16_t* rawAdcOut = NULL) {
    uint16_t adcValue = adc_.read_raw_u12();
    if (rawAdcOut != NULL) {
      *rawAdcOut = adcValue;
    }

    int64_t signedAdcValue = adcValue;
    if (referenceSelect_.read() == 1) {
      signedAdcValue = signedAdcValue - adcDivIntercept_;
    }
    if (measureSelect_.read() == 0) {  // 1:100 divider
      return signedAdcValue * 1000 * kCalibrationDenominator / adcSlope100_;
    } else {  // direct input
      return signedAdcValue * 1000 * kCalibrationDenominator / adcSlope1_;
    }
  }

protected:
  Mcp3201 adc_;
  DigitalOut measureSelect_;  // 0 = 1M/10k divider, 1: direct input
  DigitalOut referenceSelect_;  // 0 = GND, 1 = 1/2 divider (allows measuring negative voltages, 'virtual ground')

  static constexpr float kVref = 3.3;
  static const int32_t kAdcCounts = 4095;
  static const int32_t kCalibrationDenominator = 1000;
  int32_t adcDivIntercept_ = kAdcCounts / 2;

  int32_t adcSlope1_ = (float)kCalibrationDenominator * kAdcCounts / kVref;
  int32_t adcSlope100_ = (float)kCalibrationDenominator * kAdcCounts * (10.0/1010.0) / kVref;
};

#endif
