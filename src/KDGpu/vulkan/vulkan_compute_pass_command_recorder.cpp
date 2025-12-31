/*
  This file is part of KDGpu.

  SPDX-FileCopyrightText: 2023 Klarälvdalens Datakonsult AB, a KDAB Group company <info@kdab.com>

  SPDX-License-Identifier: MIT

  Contact KDAB at <info@kdab.com> for commercial licensing options.
*/

#include "vulkan_compute_pass_command_recorder.h"
#include <KDGpu/vulkan/vulkan_compute_pipeline.h>
#include <KDGpu/vulkan/vulkan_resource_manager.h>
#include <KDGpu/vulkan/vulkan_enums.h>
#include <KDGpu/vulkan/vulkan_device.h>
#include <KDGpu/vulkan/vulkan_shader_object.h>

namespace KDGpu {

VulkanComputePassCommandRecorder::VulkanComputePassCommandRecorder(VkCommandBuffer _commandBuffer,
                                                                   VulkanResourceManager *_vulkanResourceManager,
                                                                   const Handle<Device_t> &_deviceHandle)
    : commandBuffer(_commandBuffer)
    , vulkanResourceManager(_vulkanResourceManager)
    , deviceHandle(_deviceHandle)
{
}

void VulkanComputePassCommandRecorder::setPipeline(const Handle<ComputePipeline_t> &_pipeline)
{
    pipeline = _pipeline;
    pipelineLayout = {};
    VulkanComputePipeline *vulkanPipeline = vulkanResourceManager->getComputePipeline(pipeline);
    if (!vulkanPipeline)
        return;
    if (vulkanResourceManager->getPipelineLayout(vulkanPipeline->pipelineLayoutHandle)) {
        pipelineLayout = vulkanPipeline->pipelineLayoutHandle;
    }
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, vulkanPipeline->pipeline);
}

void VulkanComputePassCommandRecorder::setPipelineLayout(const Handle<PipelineLayout_t> &_pipelineLayout)
{
    pipelineLayout = _pipelineLayout;
}

void VulkanComputePassCommandRecorder::bindShader(const Handle<ShaderObject_t> &shader)
{
#if defined(VK_EXT_shader_object)
    VulkanDevice *device = vulkanResourceManager->getDevice(deviceHandle);
    if (device->vkCmdBindShadersEXT) {
        const VulkanShaderObject *shaderObject = vulkanResourceManager->getShaderObject(shader);
        assert(shaderObject && "Invalid ShaderObject provided to bindShader");
        const VkShaderStageFlagBits stage = VK_SHADER_STAGE_COMPUTE_BIT;
        device->vkCmdBindShadersEXT(commandBuffer, 1, &stage, &shaderObject->shaderObject);
    } else {
        (void)shader;
        assert(false && "VK_EXT_shader_object not enabled on device");
    }
#else
    (void)shader;
    assert(false && "VK_EXT_shader_object not supported by headers");
#endif
}

void VulkanComputePassCommandRecorder::setBindGroup(uint32_t group, const Handle<BindGroup_t> &_bindGroup,
                                                    const Handle<PipelineLayout_t> &pipelineLayout,
                                                    std::span<const uint32_t> dynamicBufferOffsets) const
{
    VulkanBindGroup *bindGroup = vulkanResourceManager->getBindGroup(_bindGroup);
    VkDescriptorSet set = bindGroup->descriptorSet;

    // Use the pipeline layout provided, otherwise fallback to the one from the currently
    // bound pipeline (if any).
    VkPipelineLayout vkPipelineLayout{ VK_NULL_HANDLE };
    if (pipelineLayout.isValid()) {
        VulkanPipelineLayout *vulkanPipelineLayout = vulkanResourceManager->getPipelineLayout(pipelineLayout);
        if (vulkanPipelineLayout != nullptr)
            vkPipelineLayout = vulkanPipelineLayout->pipelineLayout;
    } else if (this->pipelineLayout.isValid()) {
        VulkanPipelineLayout *vulkanPipelineLayout = vulkanResourceManager->getPipelineLayout(this->pipelineLayout);
        if (vulkanPipelineLayout != nullptr)
            vkPipelineLayout = vulkanPipelineLayout->pipelineLayout;
    } else if (pipeline.isValid()) {
        VulkanComputePipeline *vulkanPipeline = vulkanResourceManager->getComputePipeline(pipeline);
        if (vulkanPipeline != nullptr) {
            VulkanPipelineLayout *vulkanPipelineLayout = vulkanResourceManager->getPipelineLayout(vulkanPipeline->pipelineLayoutHandle);
            if (vulkanPipelineLayout != nullptr)
                vkPipelineLayout = vulkanPipelineLayout->pipelineLayout;
        }
    }

    assert(vkPipelineLayout != VK_NULL_HANDLE); // The PipelineLayout should outlive the pipelines

    // Bind Descriptor Set
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE,
                            vkPipelineLayout,
                            group,
                            1, &set,
                            dynamicBufferOffsets.size(), dynamicBufferOffsets.data());
}

void VulkanComputePassCommandRecorder::dispatchCompute(const ComputeCommand &command) const
{
    vkCmdDispatch(commandBuffer, command.workGroupX, command.workGroupY, command.workGroupZ);
}

void VulkanComputePassCommandRecorder::dispatchCompute(std::span<const ComputeCommand> commands) const
{
    for (const auto &c : commands)
        dispatchCompute(c);
}

void VulkanComputePassCommandRecorder::dispatchComputeIndirect(const ComputeCommandIndirect &command) const
{
    VulkanBuffer *vulkanBuffer = vulkanResourceManager->getBuffer(command.buffer);
    vkCmdDispatchIndirect(commandBuffer, vulkanBuffer->buffer, command.offset);
}

void VulkanComputePassCommandRecorder::dispatchComputeIndirect(std::span<const ComputeCommandIndirect> commands) const
{
    for (const auto &c : commands)
        dispatchComputeIndirect(c);
}

void VulkanComputePassCommandRecorder::pushConstant(const PushConstantRange &constantRange, const void *data, const Handle<PipelineLayout_t> &pipelineLayout) const
{
    VkPipelineLayout vkPipelineLayout{ VK_NULL_HANDLE };

    if (pipelineLayout.isValid()) {
        if (auto *pLayout = vulkanResourceManager->getPipelineLayout(pipelineLayout); pLayout)
            vkPipelineLayout = pLayout->pipelineLayout;
    } else if (this->pipelineLayout.isValid()) {
        if (auto *pLayout = vulkanResourceManager->getPipelineLayout(this->pipelineLayout); pLayout)
            vkPipelineLayout = pLayout->pipelineLayout;
    } else if (pipeline.isValid()) {
        VulkanComputePipeline *vulkanPipeline = vulkanResourceManager->getComputePipeline(pipeline);
        VulkanPipelineLayout *pLayout = vulkanResourceManager->getPipelineLayout(vulkanPipeline->pipelineLayoutHandle);
        if (pLayout)
            vkPipelineLayout = pLayout->pipelineLayout;
    }

    assert(vkPipelineLayout != VK_NULL_HANDLE);
    vkCmdPushConstants(commandBuffer,
                       vkPipelineLayout,
                       constantRange.shaderStages.toInt(),
                       constantRange.offset,
                       constantRange.size,
                       data);
}

void VulkanComputePassCommandRecorder::pushBindGroup(uint32_t group,
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
        } else if (this->pipelineLayout.isValid()) {
            VulkanPipelineLayout *vulkanPipelineLayout = vulkanResourceManager->getPipelineLayout(this->pipelineLayout);
            if (vulkanPipelineLayout)
                vkPipelineLayout = vulkanPipelineLayout->pipelineLayout;
        } else if (pipeline.isValid()) {
            VulkanComputePipeline *vulkanPipeline = vulkanResourceManager->getComputePipeline(pipeline);
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
                                          VK_PIPELINE_BIND_POINT_COMPUTE,
                                          vkPipelineLayout,
                                          group,
                                          writeDescriptorSets.size(),
                                          writeDescriptorSets.data());
    }
#else
    assert(false);
#endif
}

void VulkanComputePassCommandRecorder::end() const
{
    // No op
}

} // namespace KDGpu
