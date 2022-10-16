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
      control_ = (kVRef - kVDiff) / kVRef;
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

  void autoRangeResistance(int16_t voltageMv) {  // assumed input voltage is <~3v
    uint8_t currRange = getRange();

    uint16_t downRangeEstdMv = UINT16_MAX;
    if (currRange > 0) {  // if we range down, estimate the new expected voltage using R ratio
      downRangeEstdMv = voltageMv * rangeResistance_[currRange] / rangeResistance_[currRange - 1];
    }

    if (downRangeEstdMv < (int32_t)(kRangeVoltageMv * kRangeDownThreshold / 1000) && (currRange > 0)) {
      setRange(currRange - 1);
    } else if ((voltageMv > (int32_t)(kRangeVoltageMv * kRangeUpThreshold / 1000)) && (currRange < RangeCount - 1)) {
      setRange(currRange + 1);
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
  DigitalOut* const *rangeControl_;  // resistance increases (driver current reduces) as range goes up

  static constexpr float kVRef = 3.3;  // driver source voltage
  static constexpr float kVDiff = 1.0;  // target voltage across resistor
  static constexpr float kVHeadroom = 0.5;  // voltage below resistor voltage reserved for headroom (ranging threshold)

  static const uint32_t kRangeVoltageMv = (kVRef - kVDiff - kVHeadroom) * 1000;
  static const uint32_t kRangeUpThreshold = 950;  // theshold of current max voltage before we up a range
  static const uint32_t kRangeDownThreshold = 900;  // threshold of previous max voltage before we down a range
};

#endif
