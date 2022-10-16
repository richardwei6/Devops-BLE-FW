#ifndef __MEASUREMENT_WIDGET_H__
#define __MEASUREMENT_WIDGET_H__

#include "DefaultFonts.h"
#include "Widget.h"


class MeasurementWidget : public Widget {
public:
  MeasurementWidget(uint8_t kContrastActive, uint8_t kContrastBackground, uint8_t kContrastStale, uint16_t w, uint16_t h) : 
    widMeasDisp_(0, 3, 100 * 1000, FontArial32, kContrastActive, kContrastStale, FontArial16, 1000, 3),
    widMeasMode_("   ", 3, FontArial16, kContrastBackground),
    widMeasUnits_("   ", 3, FontArial16, kContrastBackground),
    widMeas_(measContents_, w, h)
    {

    }

  virtual Size layout() {
    return widMeas_.layout();
  }
  virtual void draw(GraphicsApi& gfx, uint16_t x, uint16_t y) {
    return widMeas_.draw(gfx, x, y);
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
};

#endif
