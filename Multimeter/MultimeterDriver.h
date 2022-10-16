#ifndef __MULTIMETER_DRIVER_H__
#define __MULTIMETER_DRIVER_H__
#include <mbed.h>


/**
 * Constant current driver controller for the multimeter.
 * Handles current-to-DAC conversion and auto-ranging.
 */
template <size_t RangeCount, uint8_t RangeBits>
class MultimeterDriver {
public:
  MultimeterDriver(DigitalOut& enable, PwmOut& control, 
  const uint32_t rangeResistance[RangeCount], DigitalOut* const rangeControl[RangeBits]) :
      enable_(enable), control_(control), rangeResistance_(rangeResistance), rangeControl_(rangeControl) {
    control_.period_us(50);
  }

  void enable(bool enable = true) {
    enable_ = enable;
  }

  void setCurrent(uint16_t currentUa) {
    setCurrentUa_ = currentUa;
    control_ = (float)((int64_t)setCurrentUa_ * dacSlope_ / kCalibrationDenominator + dacIntercept_) / (float)kDacCounts;

  }

  uint16_t getCurrentUa() {
    return setCurrentUa_;
  }

protected:
  DigitalOut& enable_;  // 0 = disabled, 1 = enabled
  PwmOut& control_;  // Sets the target voltage downstream of the resistor
  const uint32_t* rangeResistance_;
  DigitalOut* const *rangeControl_;
  uint16_t setCurrentUa_;

  static constexpr float kVref = 3.3;
  static constexpr float kResistance = 1000;
  static const int32_t kDacCounts = 262144;
  static const int32_t kCalibrationDenominator = 1000;

  static const int32_t dacSlope_ = (float)-kCalibrationDenominator * kDacCounts * kResistance / 1000 / 1000 / kVref;
  int32_t dacIntercept_ = kDacCounts * kVref / kVref;
};

#endif
