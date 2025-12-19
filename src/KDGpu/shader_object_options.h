#pragma once

#include <KDGpu/gpu_core.h>
#include <KDGpu/pipeline_layout_options.h>

#include <string_view>
#include <span>

namespace KDGpu {

struct ShaderObjectOptions {
    std::string_view label;
    ShaderStageFlagBits stage;
    ShaderStageFlags nextStage;
    std::span<uint32_t> code;
    std::string entryPoint;
    std::vector<Handle<BindGroupLayout_t>> bindGroupLayouts;
    std::vector<PushConstantRange> pushConstantRanges;
};

} // namespace KDGpu