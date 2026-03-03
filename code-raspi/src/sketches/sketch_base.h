#ifndef SKETCH_IF_H
#define SKETCH_IF_H

#include "../sketch_info.h"
#include <GLES2/gl2.h>
#include <vector>

class SketchBase
{
  protected:
    std::vector<GLfloat> quad;

  public:
    static GLuint compile_shader(GLenum type, const char *src);
    static void throw_shader_link_error(GLuint prog);
    static void fill_quad(std::vector<GLfloat> &quad);

    // Loads and decodes PNG; looks for file in directory of executable.
    static void load_png(uint8_t **px_arr, unsigned int *img_w, unsigned int *img_h, const char *fn);

    // Creates texture and fills with pixel data
    static GLuint create_texture(uint8_t *px_arr, unsigned w, unsigned h);

    // Creates a target texture and FBO for interim rendering
    static void create_target_texture(unsigned w, unsigned h, GLuint &tex, GLuint &fbo, GLuint &depth);

  public:
    SketchBase();
    virtual void get_info(SketchInfo &ski);
    virtual void init() = 0;
    virtual void frame(double dt) = 0;
    virtual void unload(double current_time) {};
    virtual void reload(double current_time) {};
    virtual ~SketchBase() = default;
};

#endif
