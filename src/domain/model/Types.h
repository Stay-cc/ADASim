#pragma once

#include <cstdint>

namespace adasim::domain {

/// 逻辑帧号。与仿真时钟解耦：同一帧号下的物理状态唯一，
/// 回溯定位、报文序号、指标统计都以它为基准。
using FrameId = std::uint64_t;

}  // namespace adasim::domain
