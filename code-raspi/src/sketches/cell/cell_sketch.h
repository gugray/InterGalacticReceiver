#ifndef CELL_SKETCH_H
#define CELL_SKETCH_H

#include "sketch_base.h"
#include <vector>

class CellSketch : public SketchBase
{
  private:
    const int w, h;
    const GLuint render_fbo;
    GLuint vs0 = 0;
    GLuint fs0 = 0;
    GLuint vs1 = 0;
    GLuint fs1 = 0;
    GLuint prog0 = 0;
    GLuint prog1 = 0;
    GLuint vbo = 0;
    GLuint o1_tex = 0;
    GLuint o1_fbo = 0;
    GLuint o1_depth = 0;
    double time;

  public:
    CellSketch(int w, int h, GLuint render_fbo);
    void get_info(SketchInfo &ski) override;
    virtual void init() override;
    virtual void frame(double dt) override;
    virtual void unload(double current_time) override;
    virtual void reload(double current_time) override;
};

#endif
