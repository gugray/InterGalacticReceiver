#include "sketch_frag.h"

#include "horrors.h"

// GLSL
#include "shaders.h"

// Global
#include <cstdio>

FragSketch::FragSketch(int w, int h, GLuint render_fbo, const char *frag)
    : w(w)
    , h(h)
    , render_fbo(render_fbo)
    , frag(frag)
    , time(0)
{
}

void FragSketch::init()
{
    // Compile shaders
    vs = compile_shader(GL_VERTEX_SHADER, sweep_vert);
    fs = compile_shader(GL_FRAGMENT_SHADER, frag);

    // Link program, with position attribute
    prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    const GLint ixPosAttribute = 0;
    glBindAttribLocation(prog, ixPosAttribute, "position");
    glLinkProgram(prog);
    GLint ok = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) throw_shader_link_error(prog);

    // OpenGL fidgeting
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // Array buffer: for vertex array
    glGenBuffers(1, &vbo);
}

void FragSketch::frame(double dt)
{
    time += dt;

    glUseProgram(prog);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    // This is redundant, but it's what we'll need if attributes change frame-by-frame
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * quad.size(), &quad[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

    GLint time_loc = glGetUniformLocation(prog, "time");
    GLint resolution_loc = glGetUniformLocation(prog, "resolution");

    glUniform1f(time_loc, (float)time);
    glUniform2f(resolution_loc, (float)w, (float)h);

    set_custom_uniforms(dt);

    glBindFramebuffer(GL_FRAMEBUFFER, render_fbo);
    glViewport(0, 0, w, h);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glFinish();
}

void FragSketch::unload(double current_time)
{
    glDeleteBuffers(1, &vbo);
    vbo = 0;
    glDeleteProgram(prog);
    prog = 0;
    glDeleteShader(fs);
    fs = 0;
    glDeleteShader(vs);
    vs = 0;
}

void FragSketch::reload(double current_time)
{
    time = current_time;
    init();
}
