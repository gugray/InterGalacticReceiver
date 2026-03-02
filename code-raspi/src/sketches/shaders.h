#ifndef BASE_SHADERS_H
#define BASE_SHADERS_H

constexpr const char *sweep_vert = R"(
#version 310 es
precision highp float;
layout(location = 0) in vec2 position;
void main() {
    gl_Position = vec4(position, 0.0, 1.0);
}
)";

constexpr const char *render_frag = R"(
#version 310 es
precision highp float;

uniform sampler2D tex;
uniform vec2 resolution;
uniform float time;
uniform float rand;
uniform float sketchStrength;

out vec4 fragColor;

float hash(vec2 p) {
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 78.233);
    return fract(sin(p.x + p.y) * 43758.5453);
}

vec3 whiteNoise(vec2 uv) {
    float n = hash(floor(uv * resolution.x / 2.0) + rand);
    vec3 nz = vec3(step(0.85, n));
    return nz * 0.5;
}

void main() {
    fragColor.a = 1.0;
    vec2 uv = gl_FragCoord.xy / resolution;

    if(sketchStrength == 0.0)
        fragColor.rgb = whiteNoise(uv);
    else
        fragColor.rgb = texture(tex, uv).rgb * sketchStrength;
}
)";

#endif
