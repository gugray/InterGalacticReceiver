#ifndef STAR_SKETCH_H
#define STAR_SKETCH_H

#include "sketch_frag.h"

class StarSketch : public FragSketch
{
  public:
    void get_info(SketchInfo &ski) override;
    StarSketch(int w, int h, GLuint render_fbo);
};

#endif
