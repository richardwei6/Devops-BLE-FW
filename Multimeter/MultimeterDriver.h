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
    if (enable) {
      control_ = (kVref - 1) / kVref;  // set at 1v across reference resistor when running
    } else {
      control_ = 1;  // turn off driver when not running
    }
    enable_ = enable;
  }

  uint8_t getRange() {
    uint8_t rangeIndex = 0;
    for (size_t i=0; i<RangeBits; i++) {
      rangeIndex |= (rangeControl_[i]->read() == 1) << i;
    }
    return rangeIndex;
  }

  void setRange(uint8_t rangeBits) {
    for (size_t i=0; i<RangeBits; i++) {
      rangeControl_[i]->write((rangeBits & (1 << i)) != 0);
    }
  }

  uint32_t getCurrentUa() {
    if (enable_) {
      return 1000000 / rangeResistance_[getRange()];
    } else {
      return 0;
    }
  }

protected:
  DigitalOut& enable_;  // 0 = disabled, 1 = enabled
  PwmOut& control_;  // Sets the target voltage downstream of the resistor
  const uint32_t* rangeResistance_;
  DigitalOut* const *rangeControl_;

  static constexpr float kVref = 3.3;
};

#endif
