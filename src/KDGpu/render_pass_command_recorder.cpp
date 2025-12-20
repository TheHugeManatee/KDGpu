/*
  This file is part of KDGpu.

  SPDX-FileCopyrightText: 2023 Klarälvdalens Datakonsult AB, a KDAB Group company <info@kdab.com>

  SPDX-License-Identifier: MIT

  Contact KDAB at <info@kdab.com> for commercial licensing options.
*/

#include "render_pass_command_recorder.h"

#include <KDGpu/api/graphics_api_impl.h>

namespace KDGpu {

RenderPassCommandRecorder::RenderPassCommandRecorder(GraphicsApi *api,
                                                     const Handle<Device_t> &device,
                                                     const Handle<RenderPassCommandRecorder_t> &renderPassCommandRecorder)
    : m_api(api)
    , m_device(device)
    , m_renderPassCommandRecorder(renderPassCommandRecorder)
{
}

RenderPassCommandRecorder::~RenderPassCommandRecorder()
{
    if (isValid())
        m_api->resourceManager()->deleteRenderPassCommandRecorder(handle());
}

RenderPassCommandRecorder::RenderPassCommandRecorder(RenderPassCommandRecorder &&other) noexcept
{
    m_api = std::exchange(other.m_api, nullptr);
    m_device = std::exchange(other.m_device, {});
    m_renderPassCommandRecorder = std::exchange(other.m_renderPassCommandRecorder, {});
}

RenderPassCommandRecorder &RenderPassCommandRecorder::operator=(RenderPassCommandRecorder &&other) noexcept
{
    if (this != &other) {
        if (isValid())
            m_api->resourceManager()->deleteRenderPassCommandRecorder(handle());

        m_api = std::exchange(other.m_api, nullptr);
        m_device = std::exchange(other.m_device, {});
        m_renderPassCommandRecorder = std::exchange(other.m_renderPassCommandRecorder, {});
    }
    return *this;
}

void RenderPassCommandRecorder::setPipeline(const Handle<GraphicsPipeline_t> &pipeline)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setPipeline(pipeline);
}

void RenderPassCommandRecorder::bindShaders(const std::vector<ShaderStageFlags>& stages, const std::vector<Handle<ShaderObject_t>>& shaders)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->bindShaders(stages, shaders);
}

void RenderPassCommandRecorder::setVertexBuffer(uint32_t index, const Handle<Buffer_t> &buffer, DeviceSize offset)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setVertexBuffer(index, buffer, offset);
}

void RenderPassCommandRecorder::setVertexBuffers(uint32_t firstBinding, const std::vector<VertexBufferBinding> &bindings)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setVertexBuffers(firstBinding, bindings);
}

void RenderPassCommandRecorder::setIndexBuffer(const Handle<Buffer_t> &buffer, DeviceSize offset, IndexType indexType)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setIndexBuffer(buffer, offset, indexType);
}

void RenderPassCommandRecorder::setBindGroup(uint32_t group, const Handle<BindGroup_t> &bindGroup,
                                             const Handle<PipelineLayout_t> &pipelineLayout,
                                             std::span<const uint32_t> dynamicBufferOffsets)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setBindGroup(group, bindGroup, pipelineLayout, dynamicBufferOffsets);
}

void RenderPassCommandRecorder::setViewport(const Viewport &viewport)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setViewport(viewport);
}

void RenderPassCommandRecorder::setScissor(const Rect2D &scissor)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setScissor(scissor);
}

void RenderPassCommandRecorder::setStencilReference(const StencilFaceFlags faceMask, const int reference)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setStencilReference(faceMask, reference);
}

void RenderPassCommandRecorder::setCullMode(CullModeFlags cullMode)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setCullMode(cullMode);
}
void RenderPassCommandRecorder::setDepthTestEnabled(bool enabled)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setDepthTestEnabled(enabled);
}
void RenderPassCommandRecorder::setDepthWriteEnabled(bool enabled)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setDepthWriteEnabled(enabled);
}
void RenderPassCommandRecorder::setDepthCompareOp(CompareOperation op)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setDepthCompareOp(op);
}

void RenderPassCommandRecorder::setDepthBiasEnabled(bool enabled)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setDepthBiasEnabled(enabled);
}

void RenderPassCommandRecorder::setDepthBoundsTestEnabled(bool enabled)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setDepthBoundsTestEnabled(enabled);
}

void RenderPassCommandRecorder::setDepthClampEnabled(bool enabled)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setDepthClampEnabled(enabled);
}

void RenderPassCommandRecorder::setFrontFace(FrontFace frontFace)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setFrontFace(frontFace);
}

void RenderPassCommandRecorder::setPolygonMode(PolygonMode polygonMode)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setPolygonMode(polygonMode);
}

void RenderPassCommandRecorder::setPrimitiveRestartEnabled(bool enabled)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setPrimitiveRestartEnabled(enabled);
}

void RenderPassCommandRecorder::setPrimitiveTopology(PrimitiveTopology topology)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setPrimitiveTopology(topology);
}

void RenderPassCommandRecorder::setRasterizerDiscardEnabled(bool enabled)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setRasterizerDiscardEnabled(enabled);
}

void RenderPassCommandRecorder::setTessellationDomainOrigin(TessellationDomainOrigin origin)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setTessellationDomainOrigin(origin);
}

void RenderPassCommandRecorder::setPatchControlPoints(uint32_t controlPoints)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setPatchControlPoints(controlPoints);
}

void RenderPassCommandRecorder::setRasterizationSamples(SampleCountFlagBits samples)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setRasterizationSamples(samples);
}

void RenderPassCommandRecorder::setSampleMask(SampleCountFlagBits samples, const std::vector<SampleMask> &sampleMasks)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setSampleMask(samples, sampleMasks);
}

void RenderPassCommandRecorder::setAlphaToCoverageEnabled(bool enabled)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setAlphaToCoverageEnabled(enabled);
}

void RenderPassCommandRecorder::setAlphaToOneEnabled(bool enabled)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setAlphaToOneEnabled(enabled);
}

void RenderPassCommandRecorder::setColorBlendEnabled(uint32_t firstAttachment, const std::vector<bool> &enables)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setColorBlendEnabled(firstAttachment, enables);
}

void RenderPassCommandRecorder::setColorBlendEquations(uint32_t firstAttachment, const std::vector<ColorBlendEquation> &equations)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setColorBlendEquations(firstAttachment, equations);
}

void RenderPassCommandRecorder::setColorWriteMasks(uint32_t firstAttachment, const std::vector<ColorComponentFlags> &colorWriteMasks)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setColorWriteMasks(firstAttachment, colorWriteMasks);
}

void RenderPassCommandRecorder::setLogicOp(LogicOperation op)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setLogicOp(op);
}

void RenderPassCommandRecorder::setLogicOpEnabled(bool enabled)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setLogicOpEnabled(enabled);
}

void RenderPassCommandRecorder::setStencilTestEnabled(bool enabled)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setStencilTestEnabled(enabled);
}

void RenderPassCommandRecorder::setStencilOp(StencilFaceFlags faceMask,
                                             StencilOperation failOp,
                                             StencilOperation passOp,
                                             StencilOperation depthFailOp,
                                             CompareOperation compareOp)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setStencilOp(faceMask, failOp, passOp, depthFailOp, compareOp);
}

void RenderPassCommandRecorder::setScissorWithCount(const std::vector<Rect2D> &scissors)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setScissorWithCount(scissors);
}

void RenderPassCommandRecorder::setViewportWithCount(const std::vector<Viewport> &viewports)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setViewportWithCount(viewports);
}

void RenderPassCommandRecorder::setVertexInput(const std::vector<VertexBufferLayout> &bindings,
                                               const std::vector<VertexAttribute> &attributes)
{
    auto apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setVertexInput(bindings, attributes);
}

void RenderPassCommandRecorder::end()
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->end();
}

void RenderPassCommandRecorder::draw(const DrawCommand &drawCommand)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->draw(drawCommand);
}

void RenderPassCommandRecorder::draw(std::span<const DrawCommand> drawCommands)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->draw(drawCommands);
}

void RenderPassCommandRecorder::drawIndexed(const DrawIndexedCommand &drawCommand)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->drawIndexed(drawCommand);
}

void RenderPassCommandRecorder::drawIndexed(std::span<const DrawIndexedCommand> drawCommands)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->drawIndexed(drawCommands);
}

void RenderPassCommandRecorder::drawIndirect(const DrawIndirectCommand &drawCommand)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->drawIndirect(drawCommand);
}

void RenderPassCommandRecorder::drawIndirect(std::span<const DrawIndirectCommand> drawCommands)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->drawIndirect(drawCommands);
}

void RenderPassCommandRecorder::drawIndexedIndirect(const DrawIndexedIndirectCommand &drawCommand)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->drawIndexedIndirect(drawCommand);
}

void RenderPassCommandRecorder::drawIndexedIndirect(std::span<const DrawIndexedIndirectCommand> drawCommands)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->drawIndexedIndirect(drawCommands);
}

void RenderPassCommandRecorder::drawMeshTasks(const DrawMeshCommand &drawCommand)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->drawMeshTasks(drawCommand);
}

void RenderPassCommandRecorder::drawMeshTasks(std::span<const DrawMeshCommand> drawCommands)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->drawMeshTasks(drawCommands);
}

void RenderPassCommandRecorder::drawMeshTasksIndirect(const DrawMeshIndirectCommand &drawCommand)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->drawMeshTasksIndirect(drawCommand);
}

void RenderPassCommandRecorder::drawMeshTasksIndirect(std::span<const DrawMeshIndirectCommand> drawCommands)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->drawMeshTasksIndirect(drawCommands);
}

void RenderPassCommandRecorder::pushConstant(const PushConstantRange &constantRange, const void *data, const Handle<PipelineLayout_t> &pipelineLayout)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->pushConstant(constantRange, data, pipelineLayout);
}

void RenderPassCommandRecorder::pushBindGroup(uint32_t group,
                                              std::span<const BindGroupEntry> bindGroupEntries,
                                              const Handle<PipelineLayout_t> &pipelineLayout)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->pushBindGroup(group, bindGroupEntries, pipelineLayout);
}

void RenderPassCommandRecorder::nextSubpass()
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->nextSubpass();
}

void RenderPassCommandRecorder::setInputAttachmentMapping(std::span<const uint32_t> colorAttachmentIndices,
                                                          std::optional<uint32_t> depthAttachmentIndex,
                                                          std::optional<uint32_t> stencilAttachmentIndex)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setInputAttachmentMapping(colorAttachmentIndices, depthAttachmentIndex, stencilAttachmentIndex);
}

void RenderPassCommandRecorder::setOutputAttachmentMapping(std::span<const uint32_t> remappedOutputs)
{
    auto *apiRenderPassCommandRecorder = m_api->resourceManager()->getRenderPassCommandRecorder(m_renderPassCommandRecorder);
    apiRenderPassCommandRecorder->setOutputAttachmentMapping(remappedOutputs);
}

} // namespace KDGpu
