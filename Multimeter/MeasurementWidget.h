#ifndef __MEASUREMENT_WIDGET_H__
#define __MEASUREMENT_WIDGET_H__

#include "DefaultFonts.h"
#include "Widget.h"


class MeasurementWidget : public Widget {
public:
  MeasurementWidget(uint8_t kContrastActive, uint8_t kContrastBackground, uint8_t kContrastStale, uint16_t w, uint16_t h) : 
    widDisplaySign_(signBuf_, 1, 100 * 1000, FontArial32, kContrastActive, kContrastStale),
    widDisplayInt_(intBuf_, 0, 100 * 1000, FontArial32, kContrastActive, kContrastStale),
    widDisplayFrac_(fracBuf_, 3, 100 * 1000, FontArial16, kContrastActive, kContrastStale),
    widDisplay_(widDisplayContents_, 1, true),
    widMeasMode_("   ", 3, FontArial16, kContrastBackground),
    widMeasUnits_(unitsStr_, 2, FontArial16, kContrastBackground),
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
    int32_t intpart = std::abs(value) / config_.divider;

    if (value < 0) {
      signBuf_[0] = '-'; signBuf_[1] = '\0';
    } else {
      signBuf_[0] = '\0';
    }
    widDisplaySign_.setValue(signBuf_);

    NumericTextWidget::itoa(intpart, intBuf_, 10);
    widDisplayInt_.setValue(intBuf_);

    if (config_.fractionalCount > 0) {
      fracBuf_[0] = '.';
      uint32_t fractionalMax = 10;
      for (size_t i=0; (i + 1)<config_.fractionalCount; i++) {
        fractionalMax *= 10;
      }
      int32_t fracpart = std::abs(value) % config_.divider * fractionalMax / config_.divider;
      NumericTextWidget::itoa(fracpart, fracBuf_ + 1, 10, config_.fractionalCount, '0');
    } else {
      fracBuf_[0] = '\0';
    }
    widDisplayFrac_.setValue(fracBuf_);
  }

protected:
  char signBuf_[2] = {'\0'}, intBuf_[11 + 1] = {'\0'}, fracBuf_[1 + 11 + 1] = {'\0'};

  StaleTextWidget widDisplaySign_;
  StaleTextWidget widDisplayInt_;
  StaleTextWidget widDisplayFrac_;
  Widget* widDisplayContents_[3] = {&widDisplaySign_, &widDisplayInt_, &widDisplayFrac_};
  HGridWidget<3> widDisplay_;

  TextWidget widMeasMode_;
  char unitsStr_[3] = {'\0'};
  TextWidget widMeasUnits_;

  Widget* measContents_[9] = {
      NULL, NULL, &widMeasMode_,
      NULL, NULL, NULL,
      &widDisplay_, NULL, &widMeasUnits_
  };
  FixedGridWidget widMeas_;

  Config config_;
};

#endif
