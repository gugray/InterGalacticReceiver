#ifndef MMGL01_SKETCH_H
#define MMGL01_SKETCH_H

#include "sketch_frag.h"

class MMGL01Sketch : public FragSketch
{
  public:
    void get_info(SketchInfo &ski) override;
    MMGL01Sketch(int w, int h, GLuint render_fbo);
};

#endif
