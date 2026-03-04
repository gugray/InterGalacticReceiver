#version 310 es
precision highp float;

uniform float time;
uniform vec2 resolution;
out vec4 fragColor;

float sphere(vec3 pos) {
    return length(pos) - 1.0;
}

float blob5(float d1, float d2, float d3, float d4, float d5) {
    float k = 2.0;
    return -log(exp(-k * d1) + exp(-k * d2) + exp(-k * d3) + exp(-k * d4) + exp(-k * d5)) / k;
}

float scene(vec3 pos) {
    float t = time * 0.001;

    float ec = 1.5;
    float s1 = sphere(pos - ec * vec3(cos(t * 1.1), cos(t * 1.3), cos(t * 1.7)));
    float s2 = sphere(pos + ec * vec3(cos(t * 0.7), cos(t * 1.9), cos(t * 2.3)));
    float s3 = sphere(pos + ec * vec3(cos(t * 0.3), cos(t * 2.9), sin(t * 1.1)));
    float s4 = sphere(pos + ec * vec3(sin(t * 1.3), sin(t * 1.7), sin(t * 0.7)));
    float s5 = sphere(pos + ec * vec3(sin(t * 2.3), sin(t * 1.9), sin(t * 2.9)));

    return blob5(s1, s2, s3, s4, s5);
}

float intersection(in vec3 ro, in vec3 rd) {
    const float maxd = 20.0;
    const float precis = 0.001;
    float d = precis * 2.0;
    float t = 0.0;
    for(int i = 0; i < 90; i++) {
        if(d < precis || t > maxd)
            break;
        d = scene(ro + rd * t);
        t += d;
    }

    if(t < maxd)
        return t;
    else
        return -1.0;
}

vec3 calcNormal(in vec3 pos) {
    const float eps = 0.002;

    const vec3 v1 = vec3(1.0, -1.0, -1.0);
    const vec3 v2 = vec3(-1.0, -1.0, 1.0);
    const vec3 v3 = vec3(-1.0, 1.0, -1.0);
    const vec3 v4 = vec3(1.0, 1.0, 1.0);

    return normalize(v1 * scene(pos + v1 * eps) +
        v2 * scene(pos + v2 * eps) +
        v3 * scene(pos + v3 * eps) +
        v4 * scene(pos + v4 * eps));
}

vec3 calcLight(vec3 pt, vec3 normal, vec3 clr, vec3 camDir, vec3 lPos, vec3 lClr) {
    vec3 lightdir = normalize(pt - lPos);
    float cosDiffAngle = dot(normal, -lightdir);
    float fDiff = pow(0.5 + 0.5 * cosDiffAngle, 2.5);
    float cosReflAngle = dot(-camDir, reflect(lightdir, normal));
    float fRefl = max(cosReflAngle, 0.0);

    vec3 diffuse = 1.0 * fDiff * clr;
    vec3 phong = vec3(0.5 * pow(fRefl, 64.0));

    return lClr * (diffuse + phong);
}

vec3 illuminate(in vec3 pos, in vec3 camdir) {
    vec3 nrm = calcNormal(pos);
    vec3 clr = vec3(0.5);

    // const float ETA = 0.9;
    // vec3 refrd = -refract(camdir, nrm, ETA);
    // vec3 refro = pos + 10.0 * refrd;
    // float refdist = intersection(refro, refrd);
    // vec3 refpos = refro + refdist * refrd;
    // vec3 refnormal = calcNormal(refpos);

    vec3 l1 = calcLight(pos, nrm, clr, camdir, vec3(0.0, 10.0, -20.0), vec3(1.0, 1.0, 2.0));
    vec3 l2 = calcLight(pos, nrm, clr, camdir, vec3(-20, 10.0, 0.0), vec3(3.0, 1.0, 1.0));
    vec3 l3 = calcLight(pos, nrm, clr, camdir, vec3(20.0, 10.0, 0.0), vec3(1.0, 2.0, 1.0));
    vec3 l4 = calcLight(pos, nrm, clr, camdir, vec3(0.0, -10.0, 20.0), vec3(0.6, 0.6, 0.6));
    return l1 + l2 + l3 + l4;
}

mat3 calcCamMatrix(in vec3 camPos, in vec3 lookAt) {
    vec3 ww = normalize(lookAt - camPos);
    vec3 uu = normalize(cross(ww, vec3(0.0, 1.0, 0.0)));
    vec3 vv = normalize(cross(uu, ww));
    return mat3(uu, vv, ww);
}

void main() {
    vec2 uv = (gl_FragCoord.xy - resolution.xy / 2.0) / min(resolution.x, resolution.y);

    float t = time * 0.0002;
    // t = .5800006;
    // vec3 camPos = vec3(8.0 * sin(t), 0.0, -8.0 * cos(t));
    vec3 camPos = vec3(8.0, 0.0, -8.0);
    vec3 lookAt = vec3(0.0, 0.0, 0.0);

    mat3 camMat = calcCamMatrix(camPos, lookAt);
    vec3 camDir = normalize(camMat * vec3(uv, 1.0));// z is the lens length
    vec3 camDir = normalize(camMat * vec3(uv, 1.0));// z is the lens length

    vec3 clr = vec3(0.0, 0.0, 0.0);

    float dist = intersection(camPos, camDir);
    if(dist < -0.5) {
        clr = vec3(0.1, 0.1, 0.15);
    } else {
        vec3 surfPt = camPos + dist * camDir;
        clr = illuminate(surfPt, camDir);
    }

    fragColor = vec4(clr, 1.0);
}
