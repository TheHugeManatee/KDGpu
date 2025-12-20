/*
  This file is part of KDGpu.

  SPDX-FileCopyrightText: 2025 Klarälvdalens Datakonsult AB, a KDAB Group company <info@kdab.com>

  SPDX-License-Identifier: MIT

  Contact KDAB at <info@kdab.com> for commercial licensing options.
*/

#pragma once

#include <KDGpuExample/simple_example_engine_layer.h>

#include <KDGpu/bind_group.h>
#include <KDGpu/buffer.h>
#include <KDGpu/command_buffer.h>
#include <KDGpu/pipeline_layout.h>
#include <KDGpu/pipeline_layout_options.h>
#include <KDGpu/render_pass_command_recorder_options.h>
#include <KDGpu/shader_object.h>

#include <glm/glm.hpp>

#include <array>
#include <vector>

struct ImGuiContext;

class ShaderObjectDynamicRendering : public KDGpuExample::SimpleExampleEngineLayer
{
public:
protected:
    void initializeScene() override;
    void cleanupScene() override;
    void updateScene() override;
    void render() override;
    void resize() override;
    KDGpu::AdapterAndDevice createAdapterAndDevice(const KDGpu::Surface &surface) override;

private:
    void drawControls(ImGuiContext *ctx);

    struct DynamicStateSettings {
        bool depthTestEnabled{ true };
        bool depthWriteEnabled{ true };
        bool alphaBlendEnabled{ false };
        bool wireframe{ false };
        std::array<bool, 4> colorWriteMask{ true, true, true, true };
        float saturation{ 1.0f };
        float alpha{ 1.0f };
        int cullModeIndex{ 2 }; // Default to back-face culling
        int frontFaceIndex{ 0 };
        int topologyIndex{ 0 };
        int depthCompareIndex{ 1 }; // Less
    };

    struct FragmentPushConstants {
        float saturation{ 1.0f };
        float alpha{ 1.0f };
    };

    struct DrawItem {
        uint32_t firstIndex{ 0 };
        uint32_t indexCount{ 0 };
        int32_t vertexOffset{ 0 };
    };

    enum class MeshId : uint32_t {
        Background = 0,
        RotatingTriangle,
        OverlayQuad,
        Count
    };

    KDGpu::Buffer m_vertexBuffer;
    KDGpu::Buffer m_indexBuffer;
    KDGpu::Buffer m_transformBuffer;
    void *m_transformBufferData{ nullptr };
    glm::mat4 m_transform{ 1.0f };

    KDGpu::BindGroup m_transformBindGroup;
    KDGpu::PipelineLayout m_pipelineLayout;

    KDGpu::ShaderObject m_vertexShaderObject;
    KDGpu::ShaderObject m_fragmentShaderObject;

    KDGpu::RenderPassCommandRecorderOptions m_renderPassOptions;
    KDGpu::CommandBuffer m_commandBuffer;

    std::vector<KDGpu::VertexBufferLayout> m_vertexBufferLayouts;
    std::vector<KDGpu::VertexAttribute> m_vertexAttributes;
    std::array<DrawItem, static_cast<size_t>(MeshId::Count)> m_drawItems{};
    uint32_t m_lineFirstVertex{ 0 };
    uint32_t m_lineVertexCount{ 0 };

    DynamicStateSettings m_dynamicState;
    FragmentPushConstants m_fragmentPushConstants;
    KDGpu::PushConstantRange m_fragmentPushConstantRange;
};
