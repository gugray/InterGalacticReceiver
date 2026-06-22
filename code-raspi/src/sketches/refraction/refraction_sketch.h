#ifndef RAY_SKETCH_H
#define RAY_SKETCH_H

#include "lib/q3dmath.h"
#include "sketch_frag.h"
#include <vector>

class RefractionSketch : public FragSketch
{
  private:
    uint8_t *bg_pixels = nullptr;
    unsigned int bg_w;
    unsigned int bg_h;
    GLuint bg_tex = 0;
    Vector3 cam_pos;
    Vector3 look_at;
    Matrix3 cam_mat;
    float cam_mat_arr[9];
    Matrix3 rot_mat;
    float rot_mat_arr[9];

  public:
    void calc_matrices();

  public:
    RefractionSketch(int w, int h, GLuint render_fbo);
    void get_info(SketchInfo &ski) override;
    void frame(double dt) override;
    void init() override;
    void unload(double current_time) override;
    void reload(double current_time) override;
};

#endif
