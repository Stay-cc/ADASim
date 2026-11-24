#pragma once

#include <cstdint>
#include <string>

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
    /// 决策原因（cruise / follow / emergency …）。仅用于 HUD 显示，
    /// 底座不据此改变任何行为——它要是参与了控制，算法说什么就等于底座说什么。
    std::string reason;
    /// 算法侧实测的往返时延，ms；负值表示该客户端没上报。
    /// 底座自己测不了这个量：它只知道"指令几帧之前发的"，看不到对端的处理耗时。
    double algoRttMs{-1.0};
};

}  // namespace adasim::domain
