#ifndef ANOMALY_SKETCH_H
#define ANOMALY_SKETCH_H

#include "sketch_frag.h"

class AnomalySketch : public FragSketch
{
  private:
    GLint camera_pos_loc = -1;
    GLint camera_basis_loc = -1;
    GLint hash_offset_loc = -1;
    GLuint anomaly_vbo = 0;

  public:
    AnomalySketch(int w, int h, GLuint render_fbo);
    void init() override;
    void frame(double dt) override;
    void unload(double current_time) override;
    void reload(double current_time) override;
};

#endif
