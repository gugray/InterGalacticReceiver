#version 310 es
precision highp float;

uniform sampler2D texSketch;
uniform sampler2D texOverlay;
uniform vec2 resolution;
uniform float time;
uniform float rand;
uniform int blendMode;

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

    // Only static
    if(blendMode == 0) {
        fragColor.rgb = whiteNoise(uv);
    } // Has sketch
    else {
        fragColor.rgb = texture(texSketch, uv).rgb;
    }
    // Mix in overlay
    if(blendMode == 1) {
        vec4 ovr = texture(texOverlay, uv);
        fragColor.rgb = ovr.rgb * ovr.a + fragColor.rgb * (1.0 - ovr.a);
    }
}
