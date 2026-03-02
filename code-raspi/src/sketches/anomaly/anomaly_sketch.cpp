#include "anomaly_sketch.h"

// GLSL
#include "shaders.h"

// Global
#include <cmath>

AnomalySketch::AnomalySketch(int w, int h, GLuint render_fbo)
    : FragSketch(w, h, render_fbo, anomaly_frag)
{
}

void AnomalySketch::init()
{
    vs = compile_shader(GL_VERTEX_SHADER, anomaly_vert);
    fs = compile_shader(GL_FRAGMENT_SHADER, anomaly_frag);

    prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    const GLint ixPosAttribute = 0;
    glBindAttribLocation(prog, ixPosAttribute, "position");
    glLinkProgram(prog);
    GLint ok = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) throw_shader_link_error(prog);

    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);

    glGenBuffers(1, &anomaly_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, anomaly_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * quad.size(), &quad[0], GL_STATIC_DRAW);

    glUseProgram(prog);
    camera_pos_loc = glGetUniformLocation(prog, "cameraPos");
    camera_basis_loc = glGetUniformLocation(prog, "cameraBasis");
    hash_offset_loc = glGetUniformLocation(prog, "hashOffset");
}

void AnomalySketch::frame(double dt)
{
    time += dt;

    glUseProgram(prog);

    glBindBuffer(GL_ARRAY_BUFFER, anomaly_vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

    float t = (float)time;
    float rt = sinf(t * 0.3f);
    float upx = sinf(rt);
    float upy = cosf(rt);
    float rightx = upy;
    float righty = -upx;

    glUniform3f(camera_pos_loc, t, 2.0f, 0.0f);
    glUniform4f(camera_basis_loc, upx, upy, rightx, righty);
    float h0 = t * 0.001f;
    float h1 = t * 0.001731f + 0.37f;
    glUniform2f(hash_offset_loc, h0, h1);

    glBindFramebuffer(GL_FRAMEBUFFER, render_fbo);
    glViewport(0, 0, w, h);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}

void AnomalySketch::unload(double current_time)
{
    if (anomaly_vbo != 0)
    {
        glDeleteBuffers(1, &anomaly_vbo);
        anomaly_vbo = 0;
    }
    glDeleteProgram(prog);
    prog = 0;
    glDeleteShader(fs);
    fs = 0;
    glDeleteShader(vs);
    vs = 0;
}

void AnomalySketch::reload(double current_time)
{
    time = current_time;
    init();
}
