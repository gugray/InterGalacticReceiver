#version 310 es
precision highp float;

uniform float time;
uniform vec2 resolution;
uniform sampler2D bgTex;
out vec4 fragColor;

void main() {
    vec2 uv = gl_FragCoord.xy / resolution;
    vec3 clr = texture(bgTex, uv).rgb;
    fragColor = vec4(clr, 1.0);
}
