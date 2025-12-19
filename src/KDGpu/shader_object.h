/*
  This file is part of KDGpu.

  SPDX-FileCopyrightText: 2023 Klarälvdalens Datakonsult AB, a KDAB Group company <info@kdab.com>

  SPDX-License-Identifier: MIT

  Contact KDAB at <info@kdab.com> for commercial licensing options.
*/

#pragma once

#include <KDGpu/handle.h>
#include <KDGpu/kdgpu_export.h>
#include <KDGpu/graphics_api.h>

namespace KDGpu {

struct Device_t;
struct ShaderObject_t;
struct ShaderObjectOptions;

/**
 * @brief ShaderObject
 * @ingroup public
 *
 * Shader objects for late binding shaders via VK_EXT_shader_object extension.
 */
class KDGPU_EXPORT ShaderObject
{
public:
    ShaderObject();
    ~ShaderObject();

    ShaderObject(ShaderObject &&) noexcept;
    ShaderObject &operator=(ShaderObject &&) noexcept;

    ShaderObject(const ShaderObject &) = delete;
    ShaderObject &operator=(const ShaderObject &) = delete;

    Handle<ShaderObject_t> handle() const noexcept { return m_ShaderObject; }
    bool isValid() const noexcept { return m_ShaderObject.isValid(); }

    operator Handle<ShaderObject_t>() const noexcept { return m_ShaderObject; }

private:
    ShaderObject(GraphicsApi *api, const Handle<Device_t> &device, const ShaderObjectOptions &options);

    GraphicsApi *m_api{ nullptr };
    Handle<Device_t> m_device;
    Handle<ShaderObject_t> m_ShaderObject;

    friend class Device;
};

} // namespace KDGpu
