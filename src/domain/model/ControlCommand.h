#pragma once

#include <cstdint>

#include "domain/model/Types.h"

namespace adasim::domain {

/// 算法进程回传的控制指令。
/// seq 单调递增，与 StatePacket 的 frameId 共同用于错帧检测与往返时延统计。
struct ControlCommand {
    std::uint32_t seq{0};
    FrameId echoFrameId{0};   ///< 该指令所基于的状态帧号
    double simTime{0.0};
    double steer{0.0};        ///< 前轮转角，rad
    double targetSpeed{0.0};  ///< m/s
    double targetAccel{0.0};  ///< m/s^2
};

}  // namespace adasim::domain
