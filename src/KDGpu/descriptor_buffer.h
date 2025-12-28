/*
  This file is part of KDGpu.

  SPDX-FileCopyrightText: 2024 Klarälvdalens Datakonsult AB, a KDAB Group company <info@kdab.com>

  SPDX-License-Identifier: MIT

  Contact KDAB at <info@kdab.com> for commercial licensing options.
*/

#pragma once

#include <KDGpu/gpu_core.h>
#include <KDGpu/handle.h>
#include <KDGpu/kdgpu_export.h>

#include <vector>

namespace KDGpu {

struct Buffer_t;

struct DescriptorBufferBinding {
    Handle<Buffer_t> buffer;
    DeviceSize offset{ 0 };
    BufferUsageFlags usage{ BufferUsageFlagBits::None };
};

struct DescriptorBufferBindingOffset {
    uint32_t binding{ 0 };
    DeviceSize offset{ 0 };
};

struct DescriptorBufferLayoutInfo {
    DeviceSize layoutSize{ 0 };
    std::vector<DescriptorBufferBindingOffset> bindingOffsets;
};

} // namespace KDGpu
