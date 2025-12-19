/*
  This file is part of KDGpu.

  SPDX-FileCopyrightText: 2023 Klarälvdalens Datakonsult AB, a KDAB Group company <info@kdab.com>

  SPDX-License-Identifier: MIT

  Contact KDAB at <info@kdab.com> for commercial licensing options.
*/

#include "vulkan_shader_object.h"

namespace KDGpu {

VulkanShaderObject::VulkanShaderObject(VkShaderEXT _shaderObject,
                                       VulkanResourceManager *_vulkanResourceManager,
                                       const Handle<Device_t> _deviceHandle)
    : shaderObject(_shaderObject)
    , vulkanResourceManager(_vulkanResourceManager)
    , deviceHandle(_deviceHandle)
{
}

} // namespace KDGpu
