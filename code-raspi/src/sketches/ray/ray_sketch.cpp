#include "ray_sketch.h"

// Local dependencies
#include "geo_utils.h"
#include "horrors.h"

// GLSL
#include "shaders.h"

// Global
#include <math.h>

static const char *bg_file_name = "img-tile-warm.png";
static const double fov = degToRad(60);

RaySketch::RaySketch(int w, int h, GLuint render_fbo)
    : FragSketch(w, h, render_fbo, ray_frag)
{
    load_png(&bg_pixels, &bg_w, &bg_h, bg_file_name);
}

void RaySketch::get_info(SketchInfo &ski)
{
    ski.creator = "g^b0r";
    ski.title1 = "invisible solid";
}

void RaySketch::frame(double dt)
{
    time += dt;

    // Update calculations
    calc_matrices();

    // Program to use
    glUseProgram(prog);

    // Vertex array
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * quad.size(), &quad[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

    // Uniform locations
    GLint time_loc = glGetUniformLocation(prog, "time");
    GLint resolution_loc = glGetUniformLocation(prog, "resolution");
    GLint cam_pos_loc = glGetUniformLocation(prog, "camPos");
    GLint cam_mat_loc = glGetUniformLocation(prog, "camMat");
    GLint rot_mat_loc = glGetUniformLocation(prog, "rotMat");
    GLint bg_tex_loc = glGetUniformLocation(prog, "bgTex");

    // Simple uniforms
    glUniform1f(time_loc, (float)time);
    glUniform2f(resolution_loc, (float)w, (float)h);
    glUniform3f(cam_pos_loc, cam_pos.x, cam_pos.y, cam_pos.z);
    glUniformMatrix3fv(cam_mat_loc, 1, GL_TRUE, cam_mat_arr);
    glUniformMatrix3fv(rot_mat_loc, 1, GL_TRUE, rot_mat_arr);

    // Background texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, bg_tex);
    glUniform1i(bg_tex_loc, 0);

    // Render
    glBindFramebuffer(GL_FRAMEBUFFER, render_fbo);
    glViewport(0, 0, w, h);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glFinish();
}

void RaySketch::calc_matrices()
{
    // Camera
    cam_pos = Vector3(0, 0, 10);
    look_at = Vector3(0, 0, 0);
    cam_mat = calc_cam_mat(cam_pos, look_at, fov);
    mat_to_arr(cam_mat, cam_mat_arr);

    // Object rotation
    Matrix3 rotz = calc_rotz_mat(time * 0.30);
    Matrix3 roty = calc_roty_mat(time * 0.34);
    Matrix3 rotx = calc_rotx_mat(time * 0.37);
    rot_mat = rotx * roty * rotz;
    mat_to_arr(rot_mat, rot_mat_arr);
}

void RaySketch::init()
{
    FragSketch::init();
    bg_tex = create_texture(bg_pixels, bg_w, bg_h);
}

void RaySketch::unload(double current_time)
{
    glDeleteTextures(1, &bg_tex);
    bg_tex = 0;
    FragSketch::unload(current_time);
}

void RaySketch::reload(double current_time)
{
    // This in turn calls my init() override, so nothing else to do here
    FragSketch::reload(current_time);
}
