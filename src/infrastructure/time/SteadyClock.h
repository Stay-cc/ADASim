#pragma once

#include <chrono>

#include "domain/port/IClock.h"

namespace adasim::infra {

/// 基于 steady_clock 的单调时钟。
///
/// 刻意不用 system_clock：仿真节拍与时延测量都不能被系统对时（NTP）
/// 或用户改时间影响，否则会出现"时间倒退"导致的负时延。
class SteadyClock final : public domain::IClock {
public:
    [[nodiscard]] double monotonicSeconds() const noexcept override {
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        return std::chrono::duration<double>(now).count();
    }

    [[nodiscard]] std::uint64_t monotonicNanos() const noexcept override {
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
    }
};

}  // namespace adasim::infra
