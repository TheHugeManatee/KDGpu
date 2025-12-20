/*
  This file is part of KDGpu.

  SPDX-FileCopyrightText: 2025 Klarälvdalens Datakonsult AB, a KDAB Group company <info@kdab.com>

  SPDX-License-Identifier: MIT

  Contact KDAB at <info@kdab.com> for commercial licensing options.
*/

#include "shader_object_dynamic_rendering.h"

#include <KDGpuExample/engine.h>
#include <KDGpuExample/kdgpuexample.h>

#include <KDGpu/adapter.h>
#include <KDGpu/bind_group_layout_options.h>
#include <KDGpu/bind_group_options.h>
#include <KDGpu/buffer_options.h>
#include <KDGpu/device_options.h>
#include <KDGpu/graphics_pipeline_options.h>
#include <KDGpu/shader_object_options.h>

#include <imgui.h>

#include <glm/gtx/transform.hpp>

#include <vulkan/vulkan_core.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>

using namespace KDGpu;

namespace {
struct Vertex {
    glm::vec4 position;
    glm::vec4 color;
};
static_assert(sizeof(Vertex) == 8 * sizeof(float));

constexpr std::array<const char *, 3> kCullModeLabels = { "None", "Front", "Back" };
constexpr std::array<CullModeFlags, 3> kCullModes = {
    CullModeFlags(CullModeFlagBits::None),
    CullModeFlags(CullModeFlagBits::FrontBit),
    CullModeFlags(CullModeFlagBits::BackBit)
};

constexpr std::array<const char *, 2> kFrontFaceLabels = { "Counter Clockwise", "Clockwise" };
constexpr std::array<FrontFace, 2> kFrontFaces = {
    FrontFace::CounterClockwise,
    FrontFace::Clockwise
};

constexpr std::array<const char *, 2> kTopologyLabels = { "Triangle List", "Line Strip" };
constexpr std::array<PrimitiveTopology, 2> kTopologies = {
    PrimitiveTopology::TriangleList,
    PrimitiveTopology::LineStrip
};

constexpr std::array<const char *, 6> kDepthCompareLabels = { "Never", "Less", "Equal", "Less or Equal", "Greater", "Always" };
constexpr std::array<CompareOperation, 6> kDepthCompareOps = {
    CompareOperation::Never,
    CompareOperation::Less,
    CompareOperation::Equal,
    CompareOperation::LessOrEqual,
    CompareOperation::Greater,
    CompareOperation::Always
};

ColorComponentFlags buildColorMask(const std::array<bool, 4> &maskValues)
{
    ColorComponentFlags mask = {};
    if (maskValues[0])
        mask |= ColorComponentFlagBits::RedBit;
    if (maskValues[1])
        mask |= ColorComponentFlagBits::GreenBit;
    if (maskValues[2])
        mask |= ColorComponentFlagBits::BlueBit;
    if (maskValues[3])
        mask |= ColorComponentFlagBits::AlphaBit;
    return mask;
}

} // namespace

void ShaderObjectDynamicRendering::initializeScene()
{
    const AdapterFeatures &features = m_device.adapter()->features();
    if (!features.shaderObjectDynamicRendering) {
        SPDLOG_CRITICAL("VK_EXT_shader_object dynamic rendering path is not supported on this device");
        std::abort();
    }

    registerImGuiOverlayDrawFunction([this](ImGuiContext *ctx) {
        drawControls(ctx);
    });

    std::vector<Vertex> vertexData;
    vertexData.reserve(16);
    std::vector<uint32_t> indexData;
    indexData.reserve(18);

    auto appendVertex = [&vertexData](const glm::vec3 &position, const glm::vec4 &color) {
        vertexData.push_back(Vertex{ .position = glm::vec4(position, 1.0f), .color = color });
    };

    const auto addIndexedQuad = [&](uint32_t topLeftIndex, uint32_t topRightIndex, uint32_t bottomRightIndex, uint32_t bottomLeftIndex, size_t meshIndex) {
        const uint32_t firstIndex = static_cast<uint32_t>(indexData.size());
        indexData.insert(indexData.end(),
                         { bottomLeftIndex, topLeftIndex, topRightIndex,
                           bottomLeftIndex, topRightIndex, bottomRightIndex });
        m_drawItems[meshIndex] = {
            .firstIndex = firstIndex,
            .indexCount = 6,
            .vertexOffset = 0
        };
    };

    // Background quad (farther away to highlight depth testing)
    const float backgroundZ = 0.45f;
    const glm::vec4 backgroundColor{ 0.1f, 0.25f, 0.45f, 0.35f };
    const uint32_t backgroundStart = static_cast<uint32_t>(vertexData.size());
    appendVertex({ -1.05f, -1.05f, backgroundZ }, backgroundColor); // bottom-left
    appendVertex({ -1.05f, 1.05f, backgroundZ }, backgroundColor); // top-left
    appendVertex({ 1.05f, 1.05f, backgroundZ }, backgroundColor); // top-right
    appendVertex({ 1.05f, -1.05f, backgroundZ }, backgroundColor); // bottom-right
    addIndexedQuad(backgroundStart + 1, backgroundStart + 2, backgroundStart + 3, backgroundStart + 0, static_cast<size_t>(MeshId::Background));

    // Rotating triangle slightly closer to camera
    const float triangleZ = 0.0f;
    const uint32_t triangleStart = static_cast<uint32_t>(vertexData.size());
    appendVertex({ 0.0f, -0.7f, triangleZ }, { 1.0f, 0.3f, 0.25f, 1.0f });
    appendVertex({ -0.85f, 0.6f, triangleZ }, { 0.3f, 0.85f, 1.0f, 1.0f });
    appendVertex({ 0.85f, 0.6f, triangleZ }, { 0.95f, 0.9f, 0.2f, 1.0f });
    const uint32_t triangleFirstIndex = static_cast<uint32_t>(indexData.size());
    indexData.insert(indexData.end(), { triangleStart + 0, triangleStart + 1, triangleStart + 2 });
    m_drawItems[static_cast<size_t>(MeshId::RotatingTriangle)] = {
        .firstIndex = triangleFirstIndex,
        .indexCount = 3,
        .vertexOffset = 0
    };

    // Overlay quad positioned in front of the triangle
    const float overlayZ = -0.25f;
    const glm::vec4 overlayColor{ 1.0f, 0.55f, 0.15f, 0.65f };
    const uint32_t overlayStart = static_cast<uint32_t>(vertexData.size());
    appendVertex({ -0.55f, -0.55f, overlayZ }, overlayColor);
    appendVertex({ -0.1f, 0.55f, overlayZ }, overlayColor);
    appendVertex({ 0.55f, 0.55f, overlayZ }, overlayColor);
    appendVertex({ 0.1f, -0.55f, overlayZ }, overlayColor);
    addIndexedQuad(overlayStart + 1, overlayStart + 2, overlayStart + 3, overlayStart + 0, static_cast<size_t>(MeshId::OverlayQuad));

    // Line strip overlay to highlight topology toggles.
    m_lineFirstVertex = static_cast<uint32_t>(vertexData.size());
    const glm::vec4 lineColor{ 0.95f, 0.95f, 0.95f, 1.0f };
    appendVertex({ 0.0f, 0.9f, -0.05f }, lineColor);
    appendVertex({ 0.9f, 0.0f, -0.05f }, lineColor);
    appendVertex({ 0.0f, -0.9f, -0.05f }, lineColor);
    appendVertex({ -0.9f, 0.0f, -0.05f }, lineColor);
    appendVertex({ 0.0f, 0.9f, -0.05f }, lineColor); // close the loop
    m_lineVertexCount = 5;

    {
        const BufferOptions bufferOptions = {
            .label = "ShaderObject Vertex Buffer",
            .size = vertexData.size() * sizeof(Vertex),
            .usage = BufferUsageFlagBits::VertexBufferBit | BufferUsageFlagBits::TransferDstBit,
            .memoryUsage = MemoryUsage::GpuOnly
        };
        m_vertexBuffer = m_device.createBuffer(bufferOptions);
        const BufferUploadOptions uploadOptions = {
            .destinationBuffer = m_vertexBuffer,
            .dstStages = PipelineStageFlagBit::VertexAttributeInputBit,
            .dstMask = AccessFlagBit::VertexAttributeReadBit,
            .data = vertexData.data(),
            .byteSize = vertexData.size() * sizeof(Vertex)
        };
        uploadBufferData(uploadOptions);
    }

    {
        const BufferOptions bufferOptions = {
            .label = "ShaderObject Index Buffer",
            .size = indexData.size() * sizeof(uint32_t),
            .usage = BufferUsageFlagBits::IndexBufferBit | BufferUsageFlagBits::TransferDstBit,
            .memoryUsage = MemoryUsage::GpuOnly
        };
        m_indexBuffer = m_device.createBuffer(bufferOptions);
        const BufferUploadOptions uploadOptions = {
            .destinationBuffer = m_indexBuffer,
            .dstStages = PipelineStageFlagBit::IndexInputBit,
            .dstMask = AccessFlagBit::IndexReadBit,
            .data = indexData.data(),
            .byteSize = indexData.size() * sizeof(uint32_t)
        };
        uploadBufferData(uploadOptions);
    }

    {
        const BufferOptions bufferOptions = {
            .label = "Transform UBO",
            .size = sizeof(glm::mat4),
            .usage = BufferUsageFlagBits::UniformBufferBit,
            .memoryUsage = MemoryUsage::CpuToGpu
        };
        m_transformBuffer = m_device.createBuffer(bufferOptions);
        m_transformBufferData = m_transformBuffer.map();
        m_transform = glm::mat4(1.0f);
        std::memcpy(m_transformBufferData, &m_transform, sizeof(glm::mat4));
    }

    auto vertexShaderPath = KDGpuExample::assetDir().file("shaders/examples/shader_object_dynamic_rendering/shader_object_dynamic_rendering.vert.spv");
    auto fragmentShaderPath = KDGpuExample::assetDir().file("shaders/examples/shader_object_dynamic_rendering/shader_object_dynamic_rendering.frag.spv");

    const BindGroupLayoutOptions bindGroupLayoutOptions = {
        .label = "Transform UBO Layout",
        .bindings = { {
            .binding = 0,
            .resourceType = ResourceBindingType::UniformBuffer,
            .shaderStages = ShaderStageFlags(ShaderStageFlagBits::VertexBit)
        } }
    };
    const BindGroupLayout bindGroupLayout = m_device.createBindGroupLayout(bindGroupLayoutOptions);

    m_fragmentPushConstantRange = {
        .offset = 0,
        .size = sizeof(FragmentPushConstants),
        .shaderStages = ShaderStageFlags(ShaderStageFlagBits::FragmentBit)
    };

    const PipelineLayoutOptions pipelineLayoutOptions = {
        .label = "ShaderObject Dynamic Pipeline Layout",
        .bindGroupLayouts = { bindGroupLayout },
        .pushConstantRanges = { m_fragmentPushConstantRange }
    };
    m_pipelineLayout = m_device.createPipelineLayout(pipelineLayoutOptions);

    {
        const BindGroupOptions bindGroupOptions = {
            .label = "Transform Bind Group",
            .layout = bindGroupLayout,
            .resources = { {
                .binding = 0,
                .resource = UniformBufferBinding{ .buffer = m_transformBuffer }
            } }
        };
        m_transformBindGroup = m_device.createBindGroup(bindGroupOptions);
    }

    {
        auto vertexCode = KDGpuExample::readShaderFile(vertexShaderPath);
        ShaderObjectOptions vertexOptions = {
            .label = "ShaderObject Vertex",
            .stage = ShaderStageFlagBits::VertexBit,
            .nextStage = ShaderStageFlags(ShaderStageFlagBits::FragmentBit),
            .entryPoint = "main"
        };
        vertexOptions.code = { vertexCode.data(), vertexCode.size() };
        vertexOptions.bindGroupLayouts = { bindGroupLayout.handle() };
        m_vertexShaderObject = m_device.createShaderObject(vertexOptions);
    }

    {
        auto fragmentCode = KDGpuExample::readShaderFile(fragmentShaderPath);
        ShaderObjectOptions fragmentOptions = {
            .label = "ShaderObject Fragment",
            .stage = ShaderStageFlagBits::FragmentBit,
            .nextStage = ShaderStageFlags(),
            .entryPoint = "main",
            .pushConstantRanges = { m_fragmentPushConstantRange }
        };
        fragmentOptions.code = { fragmentCode.data(), fragmentCode.size() };
        fragmentOptions.bindGroupLayouts = { bindGroupLayout.handle() };
        m_fragmentShaderObject = m_device.createShaderObject(fragmentOptions);
    }

    m_vertexBufferLayouts = { VertexBufferLayout{ .binding = 0, .stride = sizeof(Vertex) } };
    m_vertexAttributes = {
        VertexAttribute{ .location = 0, .binding = 0, .format = Format::R32G32B32A32_SFLOAT, .offset = 0 },
        VertexAttribute{ .location = 1, .binding = 0, .format = Format::R32G32B32A32_SFLOAT, .offset = sizeof(glm::vec4) }
    };

    m_renderPassOptions = {
        .colorAttachments = { {
            .view = {},
            .clearValue = { 0.15f, 0.15f, 0.20f, 1.0f },
            .finalLayout = TextureLayout::PresentSrc
        } },
        .depthStencilAttachment = {
            .view = m_depthTextureView,
            .depthLoadOperation = AttachmentLoadOperation::Clear,
            .depthStoreOperation = AttachmentStoreOperation::Store,
            .depthClearValue = 1.0f,
            .layout = TextureLayout::DepthStencilAttachmentOptimal
        }
    };
}

void ShaderObjectDynamicRendering::cleanupScene()
{
    if (m_transformBufferData) {
        m_transformBuffer.unmap();
        m_transformBufferData = nullptr;
    }

    m_vertexShaderObject = {};
    m_fragmentShaderObject = {};
    m_pipelineLayout = {};
    m_transformBindGroup = {};
    m_transformBuffer = {};
    m_vertexBuffer = {};
    m_indexBuffer = {};
    m_commandBuffer = {};
}

void ShaderObjectDynamicRendering::updateScene()
{
    static float angle = 0.0f;
    const float dt = engine()->deltaTimeSeconds();
    angle += dt * 30.0f;
    if (angle > 360.0f)
        angle -= 360.0f;

    m_transform = glm::mat4(1.0f);
    m_transform = glm::rotate(m_transform, glm::radians(angle), glm::vec3(0.0f, 0.0f, 1.0f));
    if (m_transformBufferData)
        std::memcpy(m_transformBufferData, &m_transform, sizeof(glm::mat4));
}

void ShaderObjectDynamicRendering::resize()
{
    m_renderPassOptions.depthStencilAttachment.view = m_depthTextureView;
}

AdapterAndDevice ShaderObjectDynamicRendering::createAdapterAndDevice(const Surface &surface)
{
    Adapter *selectedAdapter = m_instance.selectAdapter(AdapterDeviceType::Default);
    if (!selectedAdapter) {
        SPDLOG_CRITICAL("Unable to find a suitable Adapter for ShaderObjectDynamicRendering");
        return {};
    }

    auto queueTypes = selectedAdapter->queueTypes();
    const bool hasGraphicsAndCompute = !queueTypes.empty() && queueTypes[0].supportsFeature(QueueFlags(QueueFlagBits::GraphicsBit) | QueueFlags(QueueFlagBits::ComputeBit));
    const bool supportsPresentation = selectedAdapter->supportsPresentation(surface, 0);

    if (!hasGraphicsAndCompute || !supportsPresentation) {
        SPDLOG_CRITICAL("Selected adapter does not meet queue/presentation requirements");
        return {};
    }

    constexpr std::array<const char *, 6> requiredExtensions = {
        VK_KHR_DYNAMIC_RENDERING_EXTENSION_NAME,
        VK_EXT_EXTENDED_DYNAMIC_STATE_EXTENSION_NAME,
        VK_EXT_EXTENDED_DYNAMIC_STATE_2_EXTENSION_NAME,
        VK_EXT_EXTENDED_DYNAMIC_STATE_3_EXTENSION_NAME,
        VK_EXT_VERTEX_INPUT_DYNAMIC_STATE_EXTENSION_NAME,
        VK_EXT_SHADER_OBJECT_EXTENSION_NAME
    };

    const auto adapterExtensions = selectedAdapter->extensions();
    for (const char *extension : requiredExtensions) {
        const bool supported = std::any_of(adapterExtensions.begin(), adapterExtensions.end(), [extension](const Extension &ext) {
            return ext.name == extension;
        });
        if (!supported) {
            SPDLOG_CRITICAL("Required device extension {} is not supported by the selected adapter", extension);
            return {};
        }
    }

    DeviceOptions deviceOptions = {
        .extensions = {},
        .requestedFeatures = selectedAdapter->features()
    };
    deviceOptions.extensions.reserve(requiredExtensions.size());
    for (const char *extension : requiredExtensions)
        deviceOptions.extensions.emplace_back(extension);

    auto device = selectedAdapter->createDevice(deviceOptions);
    return { selectedAdapter, std::move(device) };
}

void ShaderObjectDynamicRendering::render()
{
    auto commandRecorder = m_device.createCommandRecorder();

    m_renderPassOptions.colorAttachments[0].view = m_swapchainViews.at(m_currentSwapchainImageIndex);
    auto renderPass = commandRecorder.beginRenderPass(m_renderPassOptions);

    const std::vector<ShaderStageFlags> shaderStages = {
        ShaderStageFlags(ShaderStageFlagBits::VertexBit),
        ShaderStageFlags(ShaderStageFlagBits::FragmentBit)
    };
    const std::vector<Handle<ShaderObject_t>> shaderHandles = {
        m_vertexShaderObject.handle(),
        m_fragmentShaderObject.handle()
    };
    renderPass.bindShaders(shaderStages, shaderHandles);

    const Viewport viewport = {
        .x = 0.0f,
        .y = 0.0f,
        .width = static_cast<float>(m_swapchainExtent.width),
        .height = static_cast<float>(m_swapchainExtent.height),
        .minDepth = 0.0f,
        .maxDepth = 1.0f
    };
    renderPass.setViewport(viewport);
    const Rect2D scissor = {
        .offset = { 0, 0 },
        .extent = { m_swapchainExtent.width, m_swapchainExtent.height }
    };
    renderPass.setScissor(scissor);

    renderPass.setVertexInput(m_vertexBufferLayouts, m_vertexAttributes);
    auto drawMesh = [&](MeshId meshId) {
        const DrawItem &item = m_drawItems[static_cast<size_t>(meshId)];
        if (item.indexCount == 0)
            return;
        const DrawIndexedCommand cmd = {
            .indexCount = item.indexCount,
            .instanceCount = 1,
            .firstIndex = item.firstIndex,
            .vertexOffset = item.vertexOffset,
            .firstInstance = 0
        };
        renderPass.drawIndexed(cmd);
    };

    renderPass.setPrimitiveTopology(PrimitiveTopology::TriangleList);
    renderPass.setCullMode(kCullModes[m_dynamicState.cullModeIndex]);
    renderPass.setFrontFace(kFrontFaces[m_dynamicState.frontFaceIndex]);
    renderPass.setPolygonMode(m_dynamicState.wireframe ? PolygonMode::Line : PolygonMode::Fill);
    renderPass.setDepthTestEnabled(m_dynamicState.depthTestEnabled);
    renderPass.setDepthWriteEnabled(m_dynamicState.depthWriteEnabled);
    renderPass.setDepthCompareOp(kDepthCompareOps[m_dynamicState.depthCompareIndex]);

    const ColorBlendEquation blendEquation = {
        .srcColorBlendFactor = BlendFactor::SrcAlpha,
        .dstColorBlendFactor = BlendFactor::OneMinusSrcAlpha,
        .colorBlendOp = BlendOperation::Add,
        .srcAlphaBlendFactor = BlendFactor::One,
        .dstAlphaBlendFactor = BlendFactor::OneMinusSrcAlpha,
        .alphaBlendOp = BlendOperation::Add
    };
    renderPass.setColorBlendEnabled(0, std::vector<bool>{ m_dynamicState.alphaBlendEnabled });
    renderPass.setColorBlendEquations(0, std::vector<ColorBlendEquation>{ blendEquation });
    renderPass.setColorWriteMasks(0, std::vector<ColorComponentFlags>{ buildColorMask(m_dynamicState.colorWriteMask) });

    renderPass.setVertexBuffer(0, m_vertexBuffer);
    renderPass.setIndexBuffer(m_indexBuffer);
    renderPass.setBindGroup(0, m_transformBindGroup, m_pipelineLayout);

    m_fragmentPushConstants.saturation = m_dynamicState.saturation;
    m_fragmentPushConstants.alpha = m_dynamicState.alpha;
    renderPass.pushConstant(m_fragmentPushConstantRange, &m_fragmentPushConstants, m_pipelineLayout);

    const bool useLineOverlay = (m_dynamicState.topologyIndex == 1);
    if (!useLineOverlay) {
        drawMesh(MeshId::OverlayQuad);
    } else if (m_lineVertexCount > 1) {
        renderPass.setPrimitiveTopology(kTopologies[m_dynamicState.topologyIndex]);
        const DrawCommand lineCmd = {
            .vertexCount = m_lineVertexCount,
            .instanceCount = 1,
            .firstVertex = m_lineFirstVertex
        };
        renderPass.draw(lineCmd);
        renderPass.setPrimitiveTopology(PrimitiveTopology::TriangleList);
    }

    drawMesh(MeshId::RotatingTriangle);
    drawMesh(MeshId::Background);

    renderImGuiOverlay(&renderPass);

    renderPass.end();
    m_commandBuffer = commandRecorder.finish();

    const SubmitOptions submitOptions = {
        .commandBuffers = { m_commandBuffer },
        .waitSemaphores = { m_presentCompleteSemaphores[m_inFlightIndex] },
        .signalSemaphores = { m_renderCompleteSemaphores[m_currentSwapchainImageIndex] }
    };
    m_queue.submit(submitOptions);
}

void ShaderObjectDynamicRendering::drawControls(ImGuiContext *ctx)
{
    ImGui::SetCurrentContext(ctx);
    if (!ImGui::Begin("Shader Objects", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }

    ImGui::TextUnformatted("Dynamic Rendering Controls");
    ImGui::Separator();
    ImGui::Combo("Topology", &m_dynamicState.topologyIndex, kTopologyLabels.data(), static_cast<int>(kTopologyLabels.size()));
    ImGui::Combo("Cull Mode", &m_dynamicState.cullModeIndex, kCullModeLabels.data(), static_cast<int>(kCullModeLabels.size()));
    ImGui::Combo("Front Face", &m_dynamicState.frontFaceIndex, kFrontFaceLabels.data(), static_cast<int>(kFrontFaceLabels.size()));
    ImGui::Combo("Depth Compare", &m_dynamicState.depthCompareIndex, kDepthCompareLabels.data(), static_cast<int>(kDepthCompareLabels.size()));

    ImGui::Checkbox("Wireframe", &m_dynamicState.wireframe);
    ImGui::Checkbox("Depth Test", &m_dynamicState.depthTestEnabled);
    ImGui::Checkbox("Depth Write", &m_dynamicState.depthWriteEnabled);
    ImGui::Checkbox("Alpha Blend", &m_dynamicState.alphaBlendEnabled);

    ImGui::TextUnformatted("Color Write Mask");
    ImGui::Checkbox("R", &m_dynamicState.colorWriteMask[0]);
    ImGui::SameLine();
    ImGui::Checkbox("G", &m_dynamicState.colorWriteMask[1]);
    ImGui::SameLine();
    ImGui::Checkbox("B", &m_dynamicState.colorWriteMask[2]);
    ImGui::SameLine();
    ImGui::Checkbox("A", &m_dynamicState.colorWriteMask[3]);

    ImGui::SliderFloat("Saturation", &m_dynamicState.saturation, 0.0f, 1.0f);
    ImGui::SliderFloat("Alpha", &m_dynamicState.alpha, 0.0f, 1.0f);

    ImGui::End();
}
