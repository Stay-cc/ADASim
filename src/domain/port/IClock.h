#pragma once

#include <cstdint>

namespace adasim::domain {

/// 墙钟时钟端口。领域层需要它来判断"这一步是否到点"，
/// 但不关心时间是来自系统时钟、单调时钟还是测试替身。
/// 实现方：infrastructure/time/SteadyClock（生产）、tests 中的 FakeClock（测试）。
struct IClock {
    virtual ~IClock() = default;

    /// 单调递增的秒数，用于步进节拍与耗时统计。
    [[nodiscard]] virtual double monotonicSeconds() const noexcept = 0;

    /// 单调递增的纳秒计数，用于高精度时延测量。
    [[nodiscard]] virtual std::uint64_t monotonicNanos() const noexcept = 0;
};

}  // namespace adasim::domain
