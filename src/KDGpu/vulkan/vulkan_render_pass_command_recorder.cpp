/*
  This file is part of KDGpu.

  SPDX-FileCopyrightText: 2023 Klarälvdalens Datakonsult AB, a KDAB Group company <info@kdab.com>

  SPDX-License-Identifier: MIT

  Contact KDAB at <info@kdab.com> for commercial licensing options.
*/

#include "vulkan_render_pass_command_recorder.h"

#include <KDGpu/vulkan/vulkan_enums.h>
#include <KDGpu/vulkan/vulkan_graphics_pipeline.h>
#include <KDGpu/vulkan/vulkan_resource_manager.h>

#include <array>

namespace KDGpu {

VulkanRenderPassCommandRecorder::VulkanRenderPassCommandRecorder(VkCommandBuffer _commandBuffer,
                                                                 VkRect2D _renderArea,
                                                                 VulkanResourceManager *_vulkanResourceManager,
                                                                 const Handle<Device_t> &_deviceHandle,
                                                                 bool _dynamicRendering)
    : commandBuffer(_commandBuffer)
    , renderArea(_renderArea)
    , vulkanResourceManager(_vulkanResourceManager)
    , deviceHandle(_deviceHandle)
    , dynamicRendering(_dynamicRendering)
{
}

void VulkanRenderPassCommandRecorder::setPipeline(const Handle<GraphicsPipeline_t> &_pipeline)
{
    pipeline = _pipeline;
    VulkanGraphicsPipeline *vulkanGraphicsPipeline = vulkanResourceManager->getGraphicsPipeline(pipeline);
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, vulkanGraphicsPipeline->pipeline);

    if (!firstPipelineWasSet) {
        // Set the initial viewport and scissor rect to the full extent of the render area
        VkViewport vkViewport = {
            .x = static_cast<float>(renderArea.offset.x),
            .y = static_cast<float>(renderArea.offset.y),
            .width = static_cast<float>(renderArea.extent.width),
            .height = static_cast<float>(renderArea.extent.height),
            .minDepth = 0.0f,
            .maxDepth = 1.0f
        };
        vkCmdSetViewport(commandBuffer, 0, 1, &vkViewport);
        vkCmdSetScissor(commandBuffer, 0, 1, &renderArea);

        firstPipelineWasSet = true;
    }
}

void VulkanRenderPassCommandRecorder::bindShaders(const std::vector<ShaderStageFlags> &stages, const std::vector<Handle<ShaderObject_t>> &shaders)
{
#ifdef VK_EXT_shader_object
    assert(stages.size() == shaders.size()); // Must have one handle per stage
    const uint32_t stageCount = static_cast<uint32_t>(shaders.size());

    // reinterpret_cast is safe because KDGpu::ShaderStageFlagBits matches VkShaderStageFlagBits exactly
    const VkShaderStageFlagBits *vkStages = reinterpret_cast<const VkShaderStageFlagBits *>(stages.data());

    // Collect all shader objects
    std::vector<VkShaderEXT> vkShaderObjects;
    vkShaderObjects.reserve(shaders.size());
    for (const auto &handle : shaders) {
        const VulkanShaderObject *shaderObject = vulkanResourceManager->getShaderObject(handle);
        assert(shaderObject && "Invalid ShaderObject handle provided to bindShaders");
        vkShaderObjects.push_back(shaderObject->shaderObject);
    }

    VulkanDevice *vulkanDevice = vulkanResourceManager->getDevice(deviceHandle);
    vulkanDevice->vkCmdBindShadersEXT(commandBuffer, stageCount, vkStages, vkShaderObjects.data());
#else
    assert(false);
#endif
}

void VulkanRenderPassCommandRecorder::setVertexBuffer(uint32_t index, const Handle<Buffer_t> &buffer, DeviceSize offset) const
{
    VulkanBuffer *vulkanBuffer = vulkanResourceManager->getBuffer(buffer);
    const std::array<VkBuffer, 1> buffers = { vulkanBuffer->buffer };
    const std::array<VkDeviceSize, 1> offsets = { offset };

    vkCmdBindVertexBuffers(commandBuffer, index, 1, buffers.data(), offsets.data());
}

void VulkanRenderPassCommandRecorder::setVertexBuffers(uint32_t firstBinding, const std::vector<VertexBufferBinding> &bindings)
{
    if (bindings.empty())
        return;

#if defined(VK_EXT_extended_dynamic_state)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdBindVertexBuffers2EXT && "VK_EXT_extended_dynamic_state not enabled on device");

    const uint32_t bindingCount = static_cast<uint32_t>(bindings.size());
    std::vector<VkBuffer> vkBuffers(bindingCount, VK_NULL_HANDLE);
    std::vector<VkDeviceSize> offsets(bindingCount);
    std::vector<VkDeviceSize> sizes(bindingCount);
    std::vector<VkDeviceSize> strides(bindingCount);

    for (uint32_t i = 0; i < bindingCount; ++i) {
        const auto &binding = bindings[i];
        VulkanBuffer *vulkanBuffer = vulkanResourceManager->getBuffer(binding.buffer);
        vkBuffers[i] = vulkanBuffer ? vulkanBuffer->buffer : VK_NULL_HANDLE;
        offsets[i] = binding.offset;
        sizes[i] = (binding.size == WholeSize) ? VK_WHOLE_SIZE : binding.size;
        strides[i] = binding.stride;
    }

    device->vkCmdBindVertexBuffers2EXT(commandBuffer,
                                       firstBinding,
                                       bindingCount,
                                       vkBuffers.data(),
                                       offsets.data(),
                                       sizes.data(),
                                       strides.data());
#else
    (void)firstBinding;
    (void)bindings;
    assert(false && "VK_EXT_extended_dynamic_state not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setIndexBuffer(const Handle<Buffer_t> &buffer, DeviceSize offset, IndexType indexType) const
{
    VulkanBuffer *vulkanBuffer = vulkanResourceManager->getBuffer(buffer);
    vkCmdBindIndexBuffer(commandBuffer, vulkanBuffer->buffer, offset, indexTypeToVkIndexType(indexType));
}

void VulkanRenderPassCommandRecorder::setBindGroup(uint32_t group, const Handle<BindGroup_t> &bindGroupH,
                                                   const Handle<PipelineLayout_t> &pipelineLayout,
                                                   std::span<const uint32_t> dynamicBufferOffsets) const
{
    VulkanBindGroup *bindGroup = vulkanResourceManager->getBindGroup(bindGroupH);
    VkDescriptorSet set = bindGroup->descriptorSet;

    // Use the pipeline layout provided, otherwise fallback to the one from the currently
    // bound pipeline (if any).
    VkPipelineLayout vkPipelineLayout{ VK_NULL_HANDLE };
    if (pipelineLayout.isValid()) {
        VulkanPipelineLayout *vulkanPipelineLayout = vulkanResourceManager->getPipelineLayout(pipelineLayout);
        if (vulkanPipelineLayout)
            vkPipelineLayout = vulkanPipelineLayout->pipelineLayout;
    } else if (pipeline.isValid()) {
        VulkanGraphicsPipeline *vulkanPipeline = vulkanResourceManager->getGraphicsPipeline(pipeline);
        if (vulkanPipeline) {
            VulkanPipelineLayout *vulkanPipelineLayout = vulkanResourceManager->getPipelineLayout(vulkanPipeline->pipelineLayoutHandle);
            if (vulkanPipelineLayout)
                vkPipelineLayout = vulkanPipelineLayout->pipelineLayout;
        }
    }

    assert(vkPipelineLayout != VK_NULL_HANDLE); // The PipelineLayout should outlive the pipelines

    // Bind Descriptor Set
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            vkPipelineLayout,
                            group,
                            1, &set,
                            dynamicBufferOffsets.size(), dynamicBufferOffsets.data());
}

void VulkanRenderPassCommandRecorder::setViewport(const Viewport &viewport) const
{
    VkViewport vkViewport = {
        .x = viewport.x,
        .y = viewport.y,
        .width = viewport.width,
        .height = viewport.height,
        .minDepth = viewport.minDepth,
        .maxDepth = viewport.maxDepth
    };
    vkCmdSetViewport(commandBuffer, 0, 1, &vkViewport);
}

void VulkanRenderPassCommandRecorder::setScissor(const Rect2D &scissor) const
{
    VkRect2D vkScissor = {
        .offset = { .x = scissor.offset.x, .y = scissor.offset.y },
        .extent = { .width = scissor.extent.width, .height = scissor.extent.height }
    };
    vkCmdSetScissor(commandBuffer, 0, 1, &vkScissor);
}

void VulkanRenderPassCommandRecorder::setStencilReference(const StencilFaceFlags faceMask, const int reference) const
{
    vkCmdSetStencilReference(commandBuffer, stencilFaceToVkStencilFace(faceMask), reference);
}
void VulkanRenderPassCommandRecorder::setCullMode(CullModeFlags cullMode)
{
#if defined(VK_EXT_extended_dynamic_state)
    if (auto *device = vulkanResourceManager->getDevice(deviceHandle); device->vkCmdSetCullModeEXT) {
        device->vkCmdSetCullModeEXT(commandBuffer, cullModeToVkCullMode(cullMode));
        return;
    }
#endif
    vkCmdSetCullMode(commandBuffer, cullModeToVkCullMode(cullMode));
}
void VulkanRenderPassCommandRecorder::setDepthTestEnabled(bool enabled)
{
#if defined(VK_EXT_extended_dynamic_state)
    if (auto *device = vulkanResourceManager->getDevice(deviceHandle); device->vkCmdSetDepthTestEnableEXT) {
        device->vkCmdSetDepthTestEnableEXT(commandBuffer, enabled ? VK_TRUE : VK_FALSE);
        return;
    }
#endif
    vkCmdSetDepthTestEnable(commandBuffer, enabled ? VK_TRUE : VK_FALSE);
}

void VulkanRenderPassCommandRecorder::setDepthWriteEnabled(bool enabled)
{
#if defined(VK_EXT_extended_dynamic_state)
    if (auto *device = vulkanResourceManager->getDevice(deviceHandle); device->vkCmdSetDepthWriteEnableEXT) {
        device->vkCmdSetDepthWriteEnableEXT(commandBuffer, enabled ? VK_TRUE : VK_FALSE);
        return;
    }
#endif
    vkCmdSetDepthWriteEnable(commandBuffer, enabled ? VK_TRUE : VK_FALSE);
}

void VulkanRenderPassCommandRecorder::setDepthCompareOp(CompareOperation op)
{
#if defined(VK_EXT_extended_dynamic_state)
    if (auto *device = vulkanResourceManager->getDevice(deviceHandle); device->vkCmdSetDepthCompareOpEXT) {
        device->vkCmdSetDepthCompareOpEXT(commandBuffer, compareOperationToVkCompareOp(op));
        return;
    }
#endif
    vkCmdSetDepthCompareOp(commandBuffer, compareOperationToVkCompareOp(op));
}

void VulkanRenderPassCommandRecorder::setDepthBiasEnabled(bool enabled)
{
#if defined(VK_EXT_extended_dynamic_state2)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetDepthBiasEnableEXT && "VK_EXT_extended_dynamic_state2 not enabled on device");
    device->vkCmdSetDepthBiasEnableEXT(commandBuffer, enabled ? VK_TRUE : VK_FALSE);
#else
    (void)enabled;
    assert(false && "VK_EXT_extended_dynamic_state2 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setDepthBoundsTestEnabled(bool enabled)
{
#if defined(VK_EXT_extended_dynamic_state)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetDepthBoundsTestEnableEXT && "VK_EXT_extended_dynamic_state not enabled on device");
    device->vkCmdSetDepthBoundsTestEnableEXT(commandBuffer, enabled ? VK_TRUE : VK_FALSE);
#else
    (void)enabled;
    assert(false && "VK_EXT_extended_dynamic_state not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setDepthClampEnabled(bool enabled)
{
#if defined(VK_EXT_extended_dynamic_state3)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetDepthClampEnableEXT && "VK_EXT_extended_dynamic_state3 not enabled on device");
    device->vkCmdSetDepthClampEnableEXT(commandBuffer, enabled ? VK_TRUE : VK_FALSE);
#else
    (void)enabled;
    assert(false && "VK_EXT_extended_dynamic_state3 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setFrontFace(FrontFace frontFace)
{
#if defined(VK_EXT_extended_dynamic_state)
    if (auto *device = vulkanResourceManager->getDevice(deviceHandle); device->vkCmdSetFrontFaceEXT) {
        device->vkCmdSetFrontFaceEXT(commandBuffer, frontFaceToVkFrontFace(frontFace));
        return;
    }
#endif
    vkCmdSetFrontFace(commandBuffer, frontFaceToVkFrontFace(frontFace));
}

void VulkanRenderPassCommandRecorder::setPolygonMode(PolygonMode polygonMode)
{
#if defined(VK_EXT_extended_dynamic_state3)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetPolygonModeEXT && "VK_EXT_extended_dynamic_state3 not enabled on device");
    device->vkCmdSetPolygonModeEXT(commandBuffer, polygonModeToVkPolygonMode(polygonMode));
#else
    (void)polygonMode;
    assert(false && "VK_EXT_extended_dynamic_state3 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setPrimitiveRestartEnabled(bool enabled)
{
#if defined(VK_EXT_extended_dynamic_state2)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetPrimitiveRestartEnableEXT && "VK_EXT_extended_dynamic_state2 not enabled on device");
    device->vkCmdSetPrimitiveRestartEnableEXT(commandBuffer, enabled ? VK_TRUE : VK_FALSE);
#else
    (void)enabled;
    assert(false && "VK_EXT_extended_dynamic_state2 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setPrimitiveTopology(PrimitiveTopology topology)
{
#if defined(VK_EXT_extended_dynamic_state)
    if (auto *device = vulkanResourceManager->getDevice(deviceHandle); device->vkCmdSetPrimitiveTopologyEXT) {
        device->vkCmdSetPrimitiveTopologyEXT(commandBuffer, primitiveTopologyToVkPrimitiveTopology(topology));
        return;
    }
#endif
    vkCmdSetPrimitiveTopology(commandBuffer, primitiveTopologyToVkPrimitiveTopology(topology));
}

void VulkanRenderPassCommandRecorder::setRasterizerDiscardEnabled(bool enabled)
{
#if defined(VK_EXT_extended_dynamic_state2)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetRasterizerDiscardEnableEXT && "VK_EXT_extended_dynamic_state2 not enabled on device");
    device->vkCmdSetRasterizerDiscardEnableEXT(commandBuffer, enabled ? VK_TRUE : VK_FALSE);
#else
    (void)enabled;
    assert(false && "VK_EXT_extended_dynamic_state2 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setTessellationDomainOrigin(TessellationDomainOrigin origin)
{
#if defined(VK_EXT_extended_dynamic_state3)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetTessellationDomainOriginEXT && "VK_EXT_extended_dynamic_state3 not enabled on device");
    device->vkCmdSetTessellationDomainOriginEXT(commandBuffer, tessellationDomainOriginToVkTessellationDomainOrigin(origin));
#else
    (void)origin;
    assert(false && "VK_EXT_extended_dynamic_state3 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setPatchControlPoints(uint32_t controlPoints)
{
#if defined(VK_EXT_extended_dynamic_state2)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetPatchControlPointsEXT && "VK_EXT_extended_dynamic_state2 not enabled on device");
    device->vkCmdSetPatchControlPointsEXT(commandBuffer, controlPoints);
#else
    (void)controlPoints;
    assert(false && "VK_EXT_extended_dynamic_state2 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setRasterizationSamples(SampleCountFlagBits samples)
{
#if defined(VK_EXT_extended_dynamic_state3)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetRasterizationSamplesEXT && "VK_EXT_extended_dynamic_state3 not enabled on device");
    device->vkCmdSetRasterizationSamplesEXT(commandBuffer, sampleCountFlagBitsToVkSampleFlagBits(samples));
#else
    (void)samples;
    assert(false && "VK_EXT_extended_dynamic_state3 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setSampleMask(SampleCountFlagBits samples, const std::vector<SampleMask> &sampleMasks)
{
#if defined(VK_EXT_extended_dynamic_state3)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetSampleMaskEXT && "VK_EXT_extended_dynamic_state3 not enabled on device");
    device->vkCmdSetSampleMaskEXT(commandBuffer, sampleCountFlagBitsToVkSampleFlagBits(samples), sampleMasks.data());
#else
    (void)samples;
    (void)sampleMasks;
    assert(false && "VK_EXT_extended_dynamic_state3 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setAlphaToCoverageEnabled(bool enabled)
{
#if defined(VK_EXT_extended_dynamic_state3)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetAlphaToCoverageEnableEXT && "VK_EXT_extended_dynamic_state3 not enabled on device");
    device->vkCmdSetAlphaToCoverageEnableEXT(commandBuffer, enabled ? VK_TRUE : VK_FALSE);
#else
    (void)enabled;
    assert(false && "VK_EXT_extended_dynamic_state3 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setAlphaToOneEnabled(bool enabled)
{
#if defined(VK_EXT_extended_dynamic_state3)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetAlphaToOneEnableEXT && "VK_EXT_extended_dynamic_state3 not enabled on device");
    device->vkCmdSetAlphaToOneEnableEXT(commandBuffer, enabled ? VK_TRUE : VK_FALSE);
#else
    (void)enabled;
    assert(false && "VK_EXT_extended_dynamic_state3 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setColorBlendEnabled(uint32_t firstAttachment, const std::vector<bool> &enables)
{
#if defined(VK_EXT_extended_dynamic_state3)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetColorBlendEnableEXT && "VK_EXT_extended_dynamic_state3 not enabled on device");

    const uint32_t count = static_cast<uint32_t>(enables.size());
    std::vector<VkBool32> vkEnables(count);
    for (uint32_t i = 0; i < count; ++i)
        vkEnables[i] = enables[i] ? VK_TRUE : VK_FALSE;

    device->vkCmdSetColorBlendEnableEXT(commandBuffer, firstAttachment, count, vkEnables.data());
#else
    (void)firstAttachment;
    (void)enables;
    assert(false && "VK_EXT_extended_dynamic_state3 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setColorBlendEquations(uint32_t firstAttachment, const std::vector<ColorBlendEquation> &equations)
{
#if defined(VK_EXT_extended_dynamic_state3)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetColorBlendEquationEXT && "VK_EXT_extended_dynamic_state3 not enabled on device");

    const uint32_t count = static_cast<uint32_t>(equations.size());
    std::vector<VkColorBlendEquationEXT> vkEquations(count);
    for (uint32_t i = 0; i < count; ++i) {
        const auto &eq = equations[i];
        vkEquations[i] = {
            .srcColorBlendFactor = blendFactorToVkBlendFactor(eq.srcColorBlendFactor),
            .dstColorBlendFactor = blendFactorToVkBlendFactor(eq.dstColorBlendFactor),
            .colorBlendOp = blendOperationToVkBlendOp(eq.colorBlendOp),
            .srcAlphaBlendFactor = blendFactorToVkBlendFactor(eq.srcAlphaBlendFactor),
            .dstAlphaBlendFactor = blendFactorToVkBlendFactor(eq.dstAlphaBlendFactor),
            .alphaBlendOp = blendOperationToVkBlendOp(eq.alphaBlendOp)
        };
    }

    device->vkCmdSetColorBlendEquationEXT(commandBuffer, firstAttachment, count, vkEquations.data());
#else
    (void)firstAttachment;
    (void)equations;
    assert(false && "VK_EXT_extended_dynamic_state3 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setColorWriteMasks(uint32_t firstAttachment, const std::vector<ColorComponentFlags> &colorWriteMasks)
{
#if defined(VK_EXT_extended_dynamic_state3)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetColorWriteMaskEXT && "VK_EXT_extended_dynamic_state3 not enabled on device");

    const uint32_t count = static_cast<uint32_t>(colorWriteMasks.size());
    std::vector<VkColorComponentFlags> masks(count);
    for (uint32_t i = 0; i < count; ++i)
        masks[i] = static_cast<VkColorComponentFlags>(colorWriteMasks[i].toInt());

    device->vkCmdSetColorWriteMaskEXT(commandBuffer, firstAttachment, count, masks.data());
#else
    (void)firstAttachment;
    (void)colorWriteMasks;
    assert(false && "VK_EXT_extended_dynamic_state3 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setLogicOp(LogicOperation op)
{
#if defined(VK_EXT_extended_dynamic_state2)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetLogicOpEXT && "VK_EXT_extended_dynamic_state2 not enabled on device");
    device->vkCmdSetLogicOpEXT(commandBuffer, logicOperationToVkLogicOp(op));
#else
    (void)op;
    assert(false && "VK_EXT_extended_dynamic_state2 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setLogicOpEnabled(bool enabled)
{
#if defined(VK_EXT_extended_dynamic_state3)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetLogicOpEnableEXT && "VK_EXT_extended_dynamic_state3 not enabled on device");
    device->vkCmdSetLogicOpEnableEXT(commandBuffer, enabled ? VK_TRUE : VK_FALSE);
#else
    (void)enabled;
    assert(false && "VK_EXT_extended_dynamic_state3 not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setStencilTestEnabled(bool enabled)
{
#if defined(VK_EXT_extended_dynamic_state)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetStencilTestEnableEXT && "VK_EXT_extended_dynamic_state not enabled on device");
    device->vkCmdSetStencilTestEnableEXT(commandBuffer, enabled ? VK_TRUE : VK_FALSE);
#else
    (void)enabled;
    assert(false && "VK_EXT_extended_dynamic_state not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setStencilOp(StencilFaceFlags faceMask,
                                                   StencilOperation failOp,
                                                   StencilOperation passOp,
                                                   StencilOperation depthFailOp,
                                                   CompareOperation compareOp)
{
#if defined(VK_EXT_extended_dynamic_state)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetStencilOpEXT && "VK_EXT_extended_dynamic_state not enabled on device");
    device->vkCmdSetStencilOpEXT(commandBuffer,
                                 stencilFaceToVkStencilFace(faceMask),
                                 stencilOperationToVkStencilOp(failOp),
                                 stencilOperationToVkStencilOp(passOp),
                                 stencilOperationToVkStencilOp(depthFailOp),
                                 compareOperationToVkCompareOp(compareOp));
#else
    (void)faceMask;
    (void)failOp;
    (void)passOp;
    (void)depthFailOp;
    (void)compareOp;
    assert(false && "VK_EXT_extended_dynamic_state not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::setScissorWithCount(const std::vector<Rect2D> &scissors)
{
    if (scissors.empty())
        return;

#if defined(VK_EXT_extended_dynamic_state)
    if (auto *device = vulkanResourceManager->getDevice(deviceHandle); device->vkCmdSetScissorWithCountEXT) {
        const uint32_t count = static_cast<uint32_t>(scissors.size());
        std::vector<VkRect2D> vkScissors;
        vkScissors.reserve(count);
        for (const auto &scissor : scissors) {
            vkScissors.push_back(VkRect2D{
                    .offset = { scissor.offset.x, scissor.offset.y },
                    .extent = { scissor.extent.width, scissor.extent.height } });
        }
        device->vkCmdSetScissorWithCountEXT(commandBuffer, count, vkScissors.data());
        return;
    }
#endif

    VkRect2D vkScissor = {
        .offset = { scissors.front().offset.x, scissors.front().offset.y },
        .extent = { scissors.front().extent.width, scissors.front().extent.height }
    };
    vkCmdSetScissor(commandBuffer, 0, 1, &vkScissor);
}

void VulkanRenderPassCommandRecorder::setViewportWithCount(const std::vector<Viewport> &viewports)
{
    if (viewports.empty())
        return;

#if defined(VK_EXT_extended_dynamic_state)
    if (auto *device = vulkanResourceManager->getDevice(deviceHandle); device->vkCmdSetViewportWithCountEXT) {
        const uint32_t count = static_cast<uint32_t>(viewports.size());
        std::vector<VkViewport> vkViewports;
        vkViewports.reserve(count);
        for (const auto &viewport : viewports) {
            vkViewports.push_back(VkViewport{
                    .x = viewport.x,
                    .y = viewport.y,
                    .width = viewport.width,
                    .height = viewport.height,
                    .minDepth = viewport.minDepth,
                    .maxDepth = viewport.maxDepth });
        }
        device->vkCmdSetViewportWithCountEXT(commandBuffer, count, vkViewports.data());
        return;
    }
#endif

    VkViewport vkViewport = {
        .x = viewports.front().x,
        .y = viewports.front().y,
        .width = viewports.front().width,
        .height = viewports.front().height,
        .minDepth = viewports.front().minDepth,
        .maxDepth = viewports.front().maxDepth
    };
    vkCmdSetViewport(commandBuffer, 0, 1, &vkViewport);
}

void VulkanRenderPassCommandRecorder::setVertexInput(const std::vector<VertexBufferLayout> &bindings,
                                                     const std::vector<VertexAttribute> &attributes)
{
#if defined(VK_EXT_vertex_input_dynamic_state)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    assert(device->vkCmdSetVertexInputEXT && "VK_EXT_vertex_input_dynamic_state not enabled on device");

    const uint32_t bindingCount = static_cast<uint32_t>(bindings.size());
    std::vector<VkVertexInputBindingDescription2EXT> vkBindings(bindingCount);
    for (uint32_t i = 0; i < bindingCount; ++i) {
        const auto &binding = bindings[i];
        vkBindings[i] = {
            .sType = VK_STRUCTURE_TYPE_VERTEX_INPUT_BINDING_DESCRIPTION_2_EXT,
            .pNext = nullptr,
            .binding = binding.binding,
            .stride = binding.stride,
            .inputRate = vertexRateToVkVertexInputRate(binding.inputRate),
            .divisor = 1
        };
    }

    const uint32_t attributeCount = static_cast<uint32_t>(attributes.size());
    std::vector<VkVertexInputAttributeDescription2EXT> vkAttributes(attributeCount);
    for (uint32_t i = 0; i < attributeCount; ++i) {
        const auto &attribute = attributes[i];
        vkAttributes[i] = {
            .sType = VK_STRUCTURE_TYPE_VERTEX_INPUT_ATTRIBUTE_DESCRIPTION_2_EXT,
            .pNext = nullptr,
            .location = attribute.location,
            .binding = attribute.binding,
            .format = formatToVkFormat(attribute.format),
            .offset = static_cast<uint32_t>(attribute.offset)
        };
    }

    device->vkCmdSetVertexInputEXT(commandBuffer,
                                   bindingCount,
                                   vkBindings.data(),
                                   attributeCount,
                                   vkAttributes.data());
#else
    (void)bindings;
    (void)attributes;
    assert(false && "VK_EXT_vertex_input_dynamic_state not supported by headers");
#endif
}

void VulkanRenderPassCommandRecorder::draw(const DrawCommand &drawCommand) const
{
    vkCmdDraw(commandBuffer,
              drawCommand.vertexCount,
              drawCommand.instanceCount,
              drawCommand.firstVertex,
              drawCommand.firstInstance);
}

void VulkanRenderPassCommandRecorder::draw(std::span<const DrawCommand> drawCommands) const
{
    for (const auto &drawCommand : drawCommands)
        draw(drawCommand);
}

void VulkanRenderPassCommandRecorder::drawIndexed(const DrawIndexedCommand &drawCommand) const
{
    vkCmdDrawIndexed(commandBuffer,
                     drawCommand.indexCount,
                     drawCommand.instanceCount,
                     drawCommand.firstIndex,
                     drawCommand.vertexOffset,
                     drawCommand.firstInstance);
}

void VulkanRenderPassCommandRecorder::drawIndexed(std::span<const DrawIndexedCommand> drawCommands) const
{
    for (const auto &drawCommand : drawCommands)
        drawIndexed(drawCommand);
}

void VulkanRenderPassCommandRecorder::drawIndirect(const DrawIndirectCommand &drawCommand) const
{
    VulkanBuffer *vulkanBuffer = vulkanResourceManager->getBuffer(drawCommand.buffer);
    vkCmdDrawIndirect(commandBuffer,
                      vulkanBuffer->buffer,
                      drawCommand.offset,
                      drawCommand.drawCount,
                      drawCommand.stride);
}

void VulkanRenderPassCommandRecorder::drawIndirect(std::span<const DrawIndirectCommand> drawCommands) const
{
    for (const auto &drawCommand : drawCommands)
        drawIndirect(drawCommand);
}

void VulkanRenderPassCommandRecorder::drawIndexedIndirect(const DrawIndexedIndirectCommand &drawCommand) const
{
    VulkanBuffer *vulkanBuffer = vulkanResourceManager->getBuffer(drawCommand.buffer);
    vkCmdDrawIndexedIndirect(commandBuffer,
                             vulkanBuffer->buffer,
                             drawCommand.offset,
                             drawCommand.drawCount,
                             drawCommand.stride);
}

void VulkanRenderPassCommandRecorder::drawIndexedIndirect(std::span<const DrawIndexedIndirectCommand> drawCommands) const
{
    for (const auto &drawCommand : drawCommands)
        drawIndexedIndirect(drawCommand);
}

void VulkanRenderPassCommandRecorder::drawMeshTasks(const DrawMeshCommand &drawCommand) const
{
#if defined(VK_EXT_mesh_shader)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    if (device->vkCmdDrawMeshTasksEXT) {
        device->vkCmdDrawMeshTasksEXT(commandBuffer,
                                      drawCommand.workGroupX,
                                      drawCommand.workGroupY,
                                      drawCommand.workGroupZ);
    }
#else
    assert(false);
#endif
}

void VulkanRenderPassCommandRecorder::drawMeshTasks(std::span<const DrawMeshCommand> drawCommands) const
{
    for (const auto &drawCommand : drawCommands)
        drawMeshTasks(drawCommand);
}

void VulkanRenderPassCommandRecorder::drawMeshTasksIndirect(const DrawMeshIndirectCommand &drawCommand) const
{
#if defined(VK_EXT_mesh_shader)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    if (device->vkCmdDrawMeshTasksIndirectEXT) {
        VulkanBuffer *vulkanBuffer = vulkanResourceManager->getBuffer(drawCommand.buffer);
        device->vkCmdDrawMeshTasksIndirectEXT(commandBuffer,
                                              vulkanBuffer->buffer,
                                              drawCommand.offset,
                                              drawCommand.drawCount,
                                              drawCommand.stride);
    }
#else
    assert(false);
#endif
}

void VulkanRenderPassCommandRecorder::drawMeshTasksIndirect(std::span<const DrawMeshIndirectCommand> drawCommands) const
{
    for (const auto &drawCommand : drawCommands)
        drawMeshTasksIndirect(drawCommand);
}

void VulkanRenderPassCommandRecorder::pushConstant(const PushConstantRange &constantRange,
                                                   const void *data,
                                                   const Handle<PipelineLayout_t> &pipelineLayout) const
{
    VkPipelineLayout vkPipelineLayout{ VK_NULL_HANDLE };

    if (pipelineLayout.isValid()) {
        VulkanPipelineLayout *vulkanPipelineLayout = vulkanResourceManager->getPipelineLayout(pipelineLayout);
        if (vulkanPipelineLayout)
            vkPipelineLayout = vulkanPipelineLayout->pipelineLayout;
    } else if (pipeline.isValid()) {
        VulkanGraphicsPipeline *vulkanPipeline = vulkanResourceManager->getGraphicsPipeline(pipeline);
        VulkanPipelineLayout *vulkanPipelineLayout = vulkanResourceManager->getPipelineLayout(vulkanPipeline->pipelineLayoutHandle);
        if (vulkanPipelineLayout)
            vkPipelineLayout = vulkanPipelineLayout->pipelineLayout;
    }

    vkCmdPushConstants(commandBuffer,
                       vkPipelineLayout,
                       constantRange.shaderStages.toInt(),
                       constantRange.offset,
                       constantRange.size,
                       data);
}

void VulkanRenderPassCommandRecorder::pushBindGroup(uint32_t group,
                                                    std::span<const BindGroupEntry> bindGroupEntries,
                                                    const Handle<PipelineLayout_t> &pipelineLayout) const
{
#if defined(VK_KHR_push_descriptor)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    if (device->vkCmdPushDescriptorSetKHR) {

        VkPipelineLayout vkPipelineLayout{ VK_NULL_HANDLE };

        if (pipelineLayout.isValid()) {
            VulkanPipelineLayout *vulkanPipelineLayout = vulkanResourceManager->getPipelineLayout(pipelineLayout);
            if (vulkanPipelineLayout)
                vkPipelineLayout = vulkanPipelineLayout->pipelineLayout;
        } else if (pipeline.isValid()) {
            VulkanGraphicsPipeline *vulkanPipeline = vulkanResourceManager->getGraphicsPipeline(pipeline);
            VulkanPipelineLayout *vulkanPipelineLayout = vulkanResourceManager->getPipelineLayout(vulkanPipeline->pipelineLayoutHandle);
            if (vulkanPipelineLayout)
                vkPipelineLayout = vulkanPipelineLayout->pipelineLayout;
        }

        std::vector<WriteBindGroupData> writeBindGroupData;
        std::vector<VkWriteDescriptorSet> writeDescriptorSets;
        const size_t bindGroupEntryCount = bindGroupEntries.size();
        writeBindGroupData.resize(bindGroupEntryCount);
        writeDescriptorSets.resize(bindGroupEntryCount);
        for (size_t i = 0; i < bindGroupEntryCount; ++i) {
            WriteBindGroupData &writeData = writeBindGroupData[i];
            device->fillWriteBindGroupDataForBindGroupEntry(writeData, bindGroupEntries[i]);
            writeDescriptorSets[i] = writeData.descriptorWrite;
        }

        device->vkCmdPushDescriptorSetKHR(commandBuffer,
                                          VK_PIPELINE_BIND_POINT_GRAPHICS,
                                          vkPipelineLayout,
                                          group,
                                          writeDescriptorSets.size(),
                                          writeDescriptorSets.data());
    }
#else
    assert(false);
#endif
}

void VulkanRenderPassCommandRecorder::nextSubpass() const
{
    if (!dynamicRendering) {
        // For now we assume renderpass/subpass are always recorded inline (primary command buffer)
        vkCmdNextSubpass(commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    }
}

void VulkanRenderPassCommandRecorder::setOutputAttachmentMapping(std::span<const uint32_t> remappedOutputs) const
{
#if defined(VK_KHR_dynamic_rendering_local_read)
    assert(dynamicRendering);
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    if (device->vkCmdSetRenderingAttachmentLocationsKHR) {
        VkRenderingAttachmentLocationInfoKHR locationInfo{};
        locationInfo.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO_KHR;
        locationInfo.colorAttachmentCount = remappedOutputs.size();
        locationInfo.pColorAttachmentLocations = remappedOutputs.data();
        device->vkCmdSetRenderingAttachmentLocationsKHR(commandBuffer, &locationInfo);
    }
#else
    assert(false);
#endif
}

void VulkanRenderPassCommandRecorder::setInputAttachmentMapping(std::span<const uint32_t> colorAttachmentIndices,
                                                                std::optional<uint32_t> depthAttachmentIndex,
                                                                std::optional<uint32_t> stencilAttachmentIndex) const
{
#if defined(VK_KHR_dynamic_rendering_local_read)
    assert(dynamicRendering);
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    if (device->vkCmdSetRenderingInputAttachmentIndicesKHR) {
        const uint32_t depthInputLocationIdx = depthAttachmentIndex ? *depthAttachmentIndex : VK_ATTACHMENT_UNUSED;
        const uint32_t stencilInputLocationIdx = stencilAttachmentIndex ? *stencilAttachmentIndex : VK_ATTACHMENT_UNUSED;

        VkRenderingInputAttachmentIndexInfoKHR locationInfo{};
        locationInfo.sType = VK_STRUCTURE_TYPE_RENDERING_INPUT_ATTACHMENT_INDEX_INFO_KHR;
        locationInfo.colorAttachmentCount = colorAttachmentIndices.size();
        locationInfo.pColorAttachmentInputIndices = colorAttachmentIndices.data();
        locationInfo.pDepthInputAttachmentIndex = &depthInputLocationIdx;
        locationInfo.pStencilInputAttachmentIndex = &stencilInputLocationIdx;

        device->vkCmdSetRenderingInputAttachmentIndicesKHR(commandBuffer, &locationInfo);
    }
#else
    assert(false);
#endif
}

void VulkanRenderPassCommandRecorder::end() const
{
    if (dynamicRendering) {
#if defined(VK_KHR_dynamic_rendering)
        VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
        device->vkCmdEndRenderingKHR(commandBuffer);
#endif
    } else {
        vkCmdEndRenderPass(commandBuffer);
    }
}

} // namespace KDGpu
