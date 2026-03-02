#version 310 es
precision highp float;
layout(location = 0) in vec2 position;

out vec2 vXZ;

const float aspectRatio = 720.0 / 576.0;
const float invCornerRadius = 1.0 / 1.60078125;

void main()
{
    gl_Position = vec4(position, 0.0, 1.0);
    // Precompute normalized dome coordinates in VS and interpolate.
    vXZ = vec2(position.x * aspectRatio, position.y) * invCornerRadius;
}
