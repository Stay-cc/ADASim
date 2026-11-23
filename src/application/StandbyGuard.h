#pragma once

#include <cstdint>

#include "domain/model/ControlCommand.h"
#include "domain/model/WorldState.h"

namespace adasim::app {

/// 断线兜底的标定参数。
///
/// 目标车距取时距式：停住时留 standstillGap，行进中每 1 m/s 再加 headway 秒，
/// 这也是跟驰模型里"安全车距"的常见形式。上限 6 m/s^2 与算法侧 emergency 档
/// 一致，再大就超出轮胎附着而不是控制问题了。
struct StandbyGuardParams {
    double standstillGap{3.0};    ///< m，停车时保留的净距
    double headway{1.5};          ///< s，每 1 m/s 车速追加的净距
    double gainGap{0.35};         ///< 1/s^2，车距误差增益
    double gainClosing{1.0};      ///< 1/s，接近速度增益（阻尼项）
    double hardTtc{1.2};          ///< s，进入"按还能停住"的硬制动层
    double maxDecel{6.0};         ///< m/s^2，兜底上限
    double engageDecel{0.5};      ///< m/s^2，低于此值才算"在制动"（状态位）
    double releaseDecel{0.15};    ///< m/s^2，回到此值以上就撤销状态位
    double jerkLimit{8.0};        ///< m/s^3，指令变化率上限
    double lateralTolerance{2.5}; ///< m，走廊半宽
};

/// 算法进程缺席时的最后一道防线。
///
/// 只在"无客户端在线"的断线保持模式下介入。算法进程一旦接入，纵向控制权
/// 完全归它——底座不能替算法踩刹车。否则感知失效、决策振荡这类本该暴露的
/// 问题会被兜底逻辑悄悄抹平，闭环对比实验就失去意义了。
///
/// 控制律是连续的两层，没有"触发即满刹"的开关：
///   1. 车距层：把净距拉回 standstillGap + headway·v，比例 + 阻尼，
///      车距有余量时输出为 0，逼近时连续增大。
///   2. 硬制动层：TTC 进到 hardTtc 以内时，改用"以现有净距刚好停住"所需的
///      减速度，只在真的要撞上时才压过第一层。
/// 两层输出取更狠者，再经加加速度限制落到指令上。
///
/// 之所以不用"阈值 + 锁存"：锁存能压住点刹，但一旦介入就恒定满刹，
/// 前后车都难受；而纯阈值判断每帧重算会抖成点刹。连续律 + 变化率限制
/// 同时解决这两件事——介入与退出都是斜坡。
class StandbyGuard {
public:
    explicit StandbyGuard(StandbyGuardParams params = {});

    /// 就地改写指令的纵向分量（转角不动，滑行时方向盘保持原位）。
    /// dt 用于限制指令变化率，必须与物理步长一致。
    /// 返回 true 表示本帧仍在制动。
    bool apply(const domain::WorldState& state, domain::ControlCommand& cmd, double dt);

    [[nodiscard]] bool engaged() const noexcept { return engaged_; }
    [[nodiscard]] double command() const noexcept { return accel_; }
    [[nodiscard]] std::uint64_t interventions() const noexcept { return interventions_; }

    /// 算法进程重新接入时调用：清介入标志与残余指令，保留累计次数。
    void release() noexcept {
        engaged_ = false;
        accel_ = 0.0;
    }

    void reset() {
        engaged_ = false;
        accel_ = 0.0;
        interventions_ = 0;
    }

private:
    StandbyGuardParams params_;
    bool engaged_{false};
    double accel_{0.0};           ///< 上一帧真正下发的减速度（负值）
    std::uint64_t interventions_{0};
};

}  // namespace adasim::app
