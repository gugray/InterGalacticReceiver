#include "render_blender.h"

// Local dependencies
#include "error.h"
#include "horrors.h"
#include "magic.h"
#include "sketches/shaders.h"
#include "sketches/sketch_base.h"

// Global
#include <stdlib.h>

RenderBlender::RenderBlender()
{
    SketchBase::create_target_texture(W, H, render_tex, render_fbo, render_depth);
    compile_render_prog();
}

void RenderBlender::render(double time)
{
    glUseProgram(render_prog);

    glBindBuffer(GL_ARRAY_BUFFER, render_vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, render_tex);

    GLint tex_loc = glGetUniformLocation(render_prog, "tex");
    GLint resolution_loc = glGetUniformLocation(render_prog, "resolution");
    GLint time_loc = glGetUniformLocation(render_prog, "time");
    GLint rand_loc = glGetUniformLocation(render_prog, "rand");
    GLint sketch_strength_loc = glGetUniformLocation(render_prog, "sketchStrength");

    glUniform1i(tex_loc, 0);
    glUniform2f(resolution_loc, (float)W, (float)H);
    glUniform1f(time_loc, (float)time);
    glUniform1f(rand_loc, (float)((double)rand() / RAND_MAX));

    float sketchStrength = 0; // static
    if (mode == bmInfo) sketchStrength = 0.2;
    else if (mode == bmSketch) sketchStrength = 1;
    glUniform1f(sketch_strength_loc, (float)sketchStrength);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, W, H);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glFinish();
}

void RenderBlender::compile_render_prog()
{
    // Compile shaders
    auto vs = SketchBase::compile_shader(GL_VERTEX_SHADER, sweep_vert);
    auto fs = SketchBase::compile_shader(GL_FRAGMENT_SHADER, render_frag);

    // Link program, with position attribute
    render_prog = glCreateProgram();
    glAttachShader(render_prog, vs);
    glAttachShader(render_prog, fs);
    const GLint ixPosAttribute = 0;
    glBindAttribLocation(render_prog, ixPosAttribute, "position");
    glLinkProgram(render_prog);
    GLint ok = 0;
    glGetProgramiv(render_prog, GL_LINK_STATUS, &ok);
    if (!ok) SketchBase::throw_shader_link_error(render_prog);

    // Array buffer: for vertex array
    std::vector<GLfloat> quad;
    SketchBase::fill_quad(quad);

    glGenBuffers(1, &render_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, render_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * quad.size(), &quad[0], GL_STATIC_DRAW);
}

void RenderBlender::set_mode(BlendMode mode)
{
    this->mode = mode;
}
