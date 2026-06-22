#include "mmgl01_sketch.h"

#include "horrors.h"

// GLSL
#include "shaders.h"

// Global

MMGL01Sketch::MMGL01Sketch(int w, int h, GLuint render_fbo)
    : FragSketch(w, h, render_fbo, mmgl01_frag)
{
}

void MMGL01Sketch::get_info(SketchInfo &ski)
{
    ski.creator = "M—|—zen";
    ski.title1 = "mmn-gl-01-240630";
}
