#ifndef RENDER_BLENDER_H
#define RENDER_BLENDER_H

#include <GLES2/gl2.h>

enum BlendMode
{
    bmStatic,
    bmInfo,
    bmSketch,
};

class RenderBlender
{
  private:
    GLuint render_tex = 0;
    GLuint render_depth = 0;
    GLuint render_fbo = 0;
    GLuint render_prog = 0;
    GLuint render_vbo = 0;
    GLuint overlay_tex = 0;
    BlendMode mode = bmStatic;

  private:
    void compile_render_prog();

  public:
    RenderBlender();
    GLuint fbo() const { return render_fbo; }
    void set_mode(BlendMode mode);
    void set_overlay(const uint8_t *px_data);
    void render(double time);
};

#endif
