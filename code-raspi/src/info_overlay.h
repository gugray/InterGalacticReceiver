#ifndef INFO_OVERLAY_H
#define INFO_OVERLAY_H

#include "lib/canvas_ity.h"
#include <stdint.h>

struct SketchInfo;

class InfoOverlay
{
  private:
    static const int buf_sz = 1024;

  private:
    const int w;
    const int h;
    char buf[buf_sz];
    size_t font_data_size;
    uint8_t *font_data;
    uint8_t *pixels;
    float *image;
    canvas_ity::canvas ctx;

  public:
    InfoOverlay(int w, int h);
    uint8_t const *render(const SketchInfo &info, uint16_t freq);
};

#endif
