/*
  This file is part of KDGpu.

  SPDX-FileCopyrightText: 2023 Klarälvdalens Datakonsult AB, a KDAB Group company <info@kdab.com>

  SPDX-License-Identifier: MIT

  Contact KDAB at <info@kdab.com> for commercial licensing options.
*/

#include "shader_object.h"

#include <KDGpu/api/graphics_api_impl.h>

namespace KDGpu {

ShaderObject::ShaderObject() = default;

ShaderObject::ShaderObject(GraphicsApi *api, const Handle<Device_t> &device, const ShaderObjectOptions &options)
    : m_api(api)
    , m_device(device)
    , m_ShaderObject(m_api->resourceManager()->createShaderObject(m_device, options))
{
}

ShaderObject::~ShaderObject()
{
    if (isValid())
        m_api->resourceManager()->deleteShaderObject(handle());
}

ShaderObject::ShaderObject(ShaderObject &&other) noexcept
{
    m_api = std::exchange(other.m_api, nullptr);
    m_device = std::exchange(other.m_device, {});
    m_ShaderObject = std::exchange(other.m_ShaderObject, {});
}

ShaderObject &ShaderObject::operator=(ShaderObject &&other) noexcept
{
    if (this != &other) {
        if (isValid())
            m_api->resourceManager()->deleteShaderObject(handle());

        m_api = std::exchange(other.m_api, nullptr);
        m_device = std::exchange(other.m_device, {});
        m_ShaderObject = std::exchange(other.m_ShaderObject, {});
    }
    return *this;
}

} // namespace KDGpu
