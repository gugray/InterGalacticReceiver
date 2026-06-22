#ifndef SKETCH_FRAG_H
#define SKETCH_FRAG_H

#include "sketch_base.h"

class FragSketch : public SketchBase
{
  protected:
    const int w, h;
    const GLuint render_fbo;
    const char *frag;
    GLuint vs = 0;
    GLuint fs = 0;
    GLuint prog = 0;
    GLuint vbo = 0;
    double time;

  protected:
    virtual void set_custom_uniforms(double dt) {};

  public:
    FragSketch(int w, int h, GLuint render_fbo, const char *frag);
    virtual void init() override;
    virtual void frame(double dt) override;
    virtual void unload(double current_time) override;
    virtual void reload(double current_time) override;
};

#endif
