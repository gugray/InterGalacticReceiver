#include "mathperc_sketch.h"

#include "horrors.h"

// GLSL
#include "shaders.h"

// Global
#include <math.h>

MathPercSketch::MathPercSketch(int w, int h, GLuint render_fbo)
    : FragSketch(w, h, render_fbo, mathperc_frag)
{
}

void MathPercSketch::get_info(SketchInfo &ski)
{
    ski.creator = "morisil";
    ski.title1 = "mathematics";
    ski.title2 = "of perception";
}

void MathPercSketch::set_custom_uniforms(double dt)
{
    GLint c1_loc = glGetUniformLocation(prog, "c1");
    GLint c2_loc = glGetUniformLocation(prog, "c2");
    GLint mat1_loc = glGetUniformLocation(prog, "mat_1");
    GLint mat2_loc = glGetUniformLocation(prog, "mat_2");

    glUniform4f(c1_loc,
                cos(time * .115) * .6 + .8,
                cos(time * .013) * 6 + 6,
                cos(time * .011) * 6 + 6,
                -(cos(time * .73) * .5 + .5) * .8 + .1);

    glUniform4f(c2_loc,
                cos(time * .15) * .2 + .2,
                .3 + cos(time * .074) * .2,
                cos(time * .081) * .6,
                cos(time * .23) * .05 + 1.1);

    float angle = cos(time * .023 + 5.) * M_PI + M_PI;
    float c = cos(angle);
    float s = sin(angle);
    float mat1[4] = {c, s, -s, c};
    glUniformMatrix2fv(mat1_loc, 1, GL_FALSE, mat1);

    angle = -cos(time * .0012) * M_PI + 1.;
    c = cos(angle);
    s = sin(angle);
    float mat2[4] = {c, s, -s, c};
    glUniformMatrix2fv(mat2_loc, 1, GL_FALSE, mat2);
}