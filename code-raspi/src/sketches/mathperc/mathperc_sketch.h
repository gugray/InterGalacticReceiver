#ifndef MATHPERC_SKETCH_H
#define MATHPERC_SKETCH_H

#include "sketch_frag.h"

class MathPercSketch : public FragSketch
{
  protected:
    void set_custom_uniforms(double dt) override;

  public:
    void get_info(SketchInfo &ski) override;
    MathPercSketch(int w, int h, GLuint render_fbo);
};

#endif
