#version 460

layout(location = 0) in vec4 inPosition;
layout(location = 1) in vec4 inColor;

layout(binding = 0) uniform Transform
{
    mat4 mvp;
} uTransform;

layout(location = 0) out vec4 vColor;

void main()
{
    gl_Position = uTransform.mvp * inPosition;
    vColor = inColor;
}
