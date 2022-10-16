#ifndef __MEASUREMENT_WIDGET_H__
#define __MEASUREMENT_WIDGET_H__

#include "DefaultFonts.h"
#include "Widget.h"


class MeasurementWidget : public Widget {
public:
  MeasurementWidget(uint8_t kContrastActive, uint8_t kContrastBackground, uint8_t kContrastStale, uint16_t w, uint16_t h) : 
    widMeasDisp_(0, 3, 100 * 1000, FontArial32, kContrastActive, kContrastStale, FontArial16, 1000, 3),
    widMeasMode_("   ", 3, FontArial16, kContrastBackground),
    widMeasUnits_("  ", 2, FontArial16, kContrastBackground),
    widMeas_(measContents_, w, h),
    config_({1, 0, 0, ' ', ' '})
    {

    }

  virtual Size layout() {
    return widMeas_.layout();
  }
  virtual void draw(GraphicsApi& gfx, uint16_t x, uint16_t y) {
    return widMeas_.draw(gfx, x, y);
  }

  struct Config {
    uint32_t divider;
    uint8_t integralCount;
    uint8_t fractionalCount;
    char prefix;
    char unit;
  };

  void setConfig(Config config) {
    unitsStr_[0] = config.prefix;
    unitsStr_[1] = config.unit;
    unitsStr_[2] = '\0';
    widMeasUnits_.setValue(unitsStr_);

    config_ = config;
  }

  void setValue(int32_t value) {
    widMeasDisp_.setValue(value / config_.divider);
  }

protected:
  StaleNumericTextWidget widMeasDisp_;
  TextWidget widMeasMode_;
  TextWidget widMeasUnits_;

  Widget* measContents_[9] = {
      NULL, NULL, &widMeasMode_,
      NULL, NULL, NULL,
      &widMeasDisp_, NULL, &widMeasUnits_
  };
  FixedGridWidget widMeas_;

  Config config_;
  char unitsStr_[3] = {'\0', '\0', '\0'};
};

#endif
