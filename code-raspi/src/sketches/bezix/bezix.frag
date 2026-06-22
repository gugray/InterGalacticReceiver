#version 310 es
precision highp float;

uniform float time;
uniform vec2 resolution;
out vec4 fragColor;

//v2
float n(float i) {
    float t = time + 1.0 * sin(time * 0.33147);
    return 3. * sin(t * (1.19 + 2.1311 * sin(i * .01776)) + i);
}
float bezier(float t, float a, float b, float c, float d) {
    float q = 1.0 - t;
    return q * q * q * n(a) +
        3. * q * q * t * n(b) +
        3. * q * t * t * n(c) +
        t * t * t * n(d);
}
float color(vec2 uv, float off) {
    vec2 a = vec2(bezier(uv.x, 1.63, -2.06 + off, 3.43, -4.88), bezier(uv.x, 9.48 + off, -8.22, 7.17, -6.11));
    vec2 b = vec2(bezier(uv.y, 5.19, 2.81, 3.73 + off, -5.18), bezier(uv.y, -1.73, -3.43, 8.71, 9.11 + off));
    return distance(a, b);
}
void main() {
    vec2 uv = gl_FragCoord.xy / resolution;
    vec3 c = vec3(color(uv, .0), color(uv, .03), color(uv, .06));
    vec3 col = vec3(1.0 / (0.1 + c * 5.0));
    fragColor = vec4(col, 1.0);
}
