#include "cell_sketch.h"

#include "horrors.h"

// GLSL
#include "shaders.h"

// Global
#include <math.h>

CellSketch::CellSketch(int w, int h, GLuint render_fbo)
    : w(w)
    , h(h)
    , render_fbo(render_fbo)
{
}

void CellSketch::get_info(SketchInfo &ski)
{
    ski.creator = "Grit Kit";
    ski.title1 = "Stellar Membrane";
}

void CellSketch::init()
{
    GLint ok;
    const GLint ixPosAttribute = 0;

    // Compile shaders
    vs0 = compile_shader(GL_VERTEX_SHADER, o_sweep_vert);
    fs0 = compile_shader(GL_FRAGMENT_SHADER, o0_frag);
    vs1 = compile_shader(GL_VERTEX_SHADER, o_sweep_vert);
    fs1 = compile_shader(GL_FRAGMENT_SHADER, o1_frag);

    // Link program 0, with position attribute
    prog0 = glCreateProgram();
    glAttachShader(prog0, vs0);
    glAttachShader(prog0, fs0);
    glBindAttribLocation(prog0, ixPosAttribute, "position");
    glLinkProgram(prog0);
    ok = 0;
    glGetProgramiv(prog0, GL_LINK_STATUS, &ok);
    if (!ok) throw_shader_link_error(prog0);

    // Link program 1, with position attribute
    prog1 = glCreateProgram();
    glAttachShader(prog1, vs1);
    glAttachShader(prog1, fs1);
    glBindAttribLocation(prog1, ixPosAttribute, "position");
    glLinkProgram(prog1);
    ok = 0;
    glGetProgramiv(prog1, GL_LINK_STATUS, &ok);
    if (!ok) throw_shader_link_error(prog1);

    // OpenGL fidgeting
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // Array buffer: for vertex array
    glGenBuffers(1, &vbo);

    // Allocate output texture for o1
    create_target_texture(w, h, o1_tex, o1_fbo, o1_depth);
}

void CellSketch::frame(double dt)
{
    time += dt;
    GLint time_loc;
    GLint resolution_loc;

    // Run program 1, render to o1_fbo
    glUseProgram(prog1);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    // This is redundant, but it's what we'll need if attributes change frame-by-frame
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * quad.size(), &quad[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

    time_loc = glGetUniformLocation(prog1, "time");
    resolution_loc = glGetUniformLocation(prog1, "resolution");

    glUniform1f(time_loc, (float)time);
    glUniform2f(resolution_loc, (float)w, (float)h);

    glBindFramebuffer(GL_FRAMEBUFFER, o1_fbo);
    glViewport(0, 0, w, h);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glFinish();

    // Run program 0, render to render_fbo
    glUseProgram(prog0);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    // This is redundant, but it's what we'll need if attributes change frame-by-frame
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * quad.size(), &quad[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

    time_loc = glGetUniformLocation(prog0, "time");
    resolution_loc = glGetUniformLocation(prog0, "resolution");
    GLint calc01_loc = glGetUniformLocation(prog0, "calc01");
    GLint calc02_loc = glGetUniformLocation(prog0, "calc02");
    GLint tex_o1_loc = glGetUniformLocation(prog0, "tex_o1");
    GLint rotate_opt_c_loc = glGetUniformLocation(prog0, "rotate_opt_c");
    GLint rotate_opt_s_loc = glGetUniformLocation(prog0, "rotate_opt_s");

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, o1_tex);

    glUniform1i(tex_o1_loc, 1);
    glUniform1f(calc01_loc, (sin(time) + 1.5) * 0.05);
    glUniform1f(calc02_loc, (sin(time * 0.5) + 1.0) * 0.12);
    glUniform1f(rotate_opt_c_loc, cos(1 + 0.1 * time));
    glUniform1f(rotate_opt_s_loc, sin(1 + 0.1 * time));
    glUniform1f(time_loc, (float)time);
    glUniform2f(resolution_loc, (float)w, (float)h);

    glBindFramebuffer(GL_FRAMEBUFFER, render_fbo);
    glViewport(0, 0, w, h);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glFinish();
}

void CellSketch::unload(double current_time)
{
    glDeleteBuffers(1, &vbo);
    vbo = 0;
    glDeleteProgram(prog1);
    prog1 = 0;
    glDeleteShader(fs1);
    fs1 = 0;
    glDeleteShader(vs1);
    vs1 = 0;
    glDeleteFramebuffers(1, &o1_fbo);
    o1_fbo = 0;
    glDeleteRenderbuffers(1, &o1_depth);
    o1_depth = 0;
    glDeleteTextures(1, &o1_tex);
    o1_tex = 0;
}

void CellSketch::reload(double current_time)
{
    time = current_time;
    init();
}
