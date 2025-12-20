#version 460

layout(location = 0) in vec4 vColor;
layout(location = 0) out vec4 outColor;

layout(push_constant) uniform PushConstants
{
    float saturation;
    float alpha;
} uPush;

vec3 applySaturation(vec3 color, float saturation)
{
    const vec3 luminanceWeights = vec3(0.299, 0.587, 0.114);
    float luminance = dot(color, luminanceWeights);
    return mix(vec3(luminance), color, saturation);
}

void main()
{
    vec3 saturated = applySaturation(vColor.rgb, uPush.saturation);
    outColor = vec4(saturated, vColor.a * uPush.alpha);
}
