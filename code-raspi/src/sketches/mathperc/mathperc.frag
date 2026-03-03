// Copyright Kazimierz Pogoda, 2022 - https://xemantic.com/
// I am the sole copyright owner of this Work.
// You cannot host, display, distribute or share this Work in any form,
// including physical and digital. You cannot use this Work in any
// commercial or non-commercial product, website or project. You cannot
// sell this Work and you cannot mint an NFTs of it.
// I share this Work for educational purposes, and you can link to it,
// through an URL, proper attribution and unmodified screenshot, as part
// of your educational material. If these conditions are too restrictive
// please contact me and we'll definitely work it out.

// copyright statement borrowed from Inigo Quilez

/*
Descripition copied from fx(hash), where this system is mintable in unique
iterations:

https://www.fxhash.xyz/generative/slug/the-mathematics-of-perception

This system, called The Mathematics of Perception, emerged from a
series of thought experiments. Our xemantic collective applies philosophy
to facts about the world. The physics behind sensory experience is already
deeply researched. But what makes certain experiences evoke certain
feelings and affectionate states in the broader sense? I want to evoke
emotions with algorithms. This research is needed for bigger immersive
installations, using certain aesthetics for telling various narratives.
However I am not a video artist, I don't cut and transform existing frames.
I synthesize them with equations. The process can be described as
sculpting in light and time with math.

Generating video-experience, which is perceptually pleasant, usually
involves 3D modeling. There is no 3D per se involved in this system, not
even so called ray marching. It represents an optical illusion of infinitive
space coded as a single GLSL fragment shader.

It started with a sketch - how to show an unlimited grid of lights,
overlapping each other in perspective and movement.

Then I added depth of field simulation, to blur the light discs depending
on perceived distance. Usually generating things "out of focus" is surprisingly
expensive to compute. Here simplicity of mathematical analytic formula
came very handy without extra cost.

After depth of field, the simulation of connected chromatic aberration
followed. This alone is the actual source of color in this system.

By accident I discovered that I can also introduce simulated refractions.
It's hard to believe how much this simple addition improved the experience.
The "refraction" is not fully following the physics of perception. It is
"impossible" on purpose, still believable optical illusion. If you feel
oniric and escheresque, probably it's thanks to this single line of code.

The waves are important as well. Usually I use trigonometric functions for
expressing motion. They have this ability of producing oscillation cycles
we observe everywhere in the physical world. From rocking in our cradles,
later observing branches of a tree when the wind blows, through experience
of music, which is unconscious perception of ratios between waves, we relate
to this kind of swinging movement. This is how we dance.

Each minted variant of this system will have different base parameters and
motion, providing similar, but unique experience. I hope that due to
differences, each of them might evoke slightly different emotions.
Therefore this drop is to please your senses, but also to continue with
our experiment. After minting each variant please write us back how you
feel out it.

Kazik Pogoda
the mother of xemantic
 */

#version 310 es
precision highp float;

uniform float time;
uniform vec2 resolution;
uniform vec4 c1;
uniform vec4 c2;
uniform mat2 mat_1;
uniform mat2 mat_2;
out vec4 fragColor;

const int ITERATIONS = 15;
const float PI = 3.14159265359;

mat2 rotate2d(in float angle) {
    return mat2(cos(angle), -sin(angle), sin(angle), cos(angle));
}

float shape(in vec2 st, in float size, in float blur) {
    vec2 modSt = mod(st, 1.) * 2. - 1.;
    float dist = length(modSt);
    return smoothstep(size + blur, size - blur, dist);
}

void main() {
    vec2 st = (2. * gl_FragCoord.xy - resolution.xy) / min(resolution.x, resolution.y);

    vec3 color = vec3(0);
    float luma = .5;
    // c1[0] = cos(time * .115) * .6 + .8
    vec2 layerSt = st * c1[0];
    // mat_1 = rotate2d(cos(time * .023 + 5.) * PI + PI);
    //       > mat2(cos(angle), -sin(angle), sin(angle), cos(angle));
    layerSt *= mat_1;

    for (int i = 0; i < ITERATIONS; i++) {
        // c1[1] = cos(time * .013) * 6. + 6.
        // c2[2] = cos(time * .011) * 6. + 6.
        vec2 gridSt = layerSt + vec2(c1[1], c1[2]);
        // mat_2 = rotate2d(-cos(time * .0012) * PI + 1.);
        //       > mat2(cos(angle), -sin(angle), sin(angle), cos(angle));
        layerSt *= mat_2;

        float depth = (float(i) + .5) / float(ITERATIONS);
        // c1[3] = -(cos(time * .73) * .5 + .5) * .8 + .1
        float focusDepth = depth + c1[3];
        float blur = .05 + focusDepth * focusDepth * .4;

        // c2[0] = cos(time * .15) * .2 + .2
        float chromaticAberration = c2[0];
        // c2[1] = .3 + cos(time * .074) * .2
        float shapeSize = c2[1];
        vec3 shapeColor = luma * vec3(
            shape(gridSt - st * chromaticAberration * blur, shapeSize, blur),
            shape(gridSt, shapeSize, blur),
            shape(gridSt + st * chromaticAberration * blur, shapeSize, blur));

        // c2[2] = cos(time * .081) * .6
        layerSt += st * shape(gridSt, shapeSize, .5) * c2[2];
        // c2[3] = cos(time * .23) * .05 + 1.1
        layerSt *= c2[3];
        color += shapeColor;
        luma *= .85;
    }
    fragColor = vec4(color, 1.0);
}
