#include "application/StandbyGuard.h"

#include <algorithm>

#include "domain/model/ForwardGap.h"

namespace adasim::app {
namespace {

constexpr double kMinClosingSpeed = 0.05;  // m/s，低于此值认为车距不再缩短
constexpr double kDefaultDt = 0.02;        // s，调用方传 0 时的兜底步长

}  // namespace

StandbyGuard::StandbyGuard(StandbyGuardParams params) : params_(params) {}

bool StandbyGuard::apply(const domain::WorldState& state, domain::ControlCommand& cmd,
                         double dt) {
    if (dt <= 0.0) {
        dt = kDefaultDt;
    }

    const domain::ForwardGap target =
        domain::nearestForwardTarget(state, params_.lateralTolerance);

    double desired = 0.0;

    if (target.valid) {
        // 第 1 层：时距式目标车距上的比例 + 阻尼控制。
        // 车距有余量时输出为正，随后被夹到 0；逼近越急，输出越负。
        const double targetGap =
            params_.standstillGap + params_.headway * state.egoSpeed;
        const double error = target.bumperGap - targetGap;
        desired = params_.gainGap * error - params_.gainClosing * target.closingSpeed;

        // 第 2 层：TTC 进到硬阈值以内，改用"以现有净距刚好停住"所需的减速度。
        // 用 usable < closing*hardTtc 表达 ttc < hardTtc，避免除零。
        const double usable = target.bumperGap - params_.standstillGap;
        if (target.closingSpeed > kMinClosingSpeed &&
            usable < target.closingSpeed * params_.hardTtc) {
            const double needed =
                usable > 0.0
                    ? -(target.closingSpeed * target.closingSpeed) / (2.0 * usable)
                    : -params_.maxDecel;
            desired = std::min(desired, needed);
        }

        desired = std::clamp(desired, -params_.maxDecel, 0.0);
    }

    // 加加速度限制：一帧之内命令不许跳变。这一步才是"不点刹"的关键——
    // 没有它，阈值一跨就会从 0 直接跳到 -3，视觉上就是一脚点刹。
    const double maxDelta = params_.jerkLimit * dt;
    accel_ += std::clamp(desired - accel_, -maxDelta, maxDelta);
    accel_ = std::min(accel_, 0.0);   // 兜底只负责制动，绝不主动加速

    cmd.targetAccel = accel_;

    // 状态位带滞后，只用于界面提示与"介入次数"统计：
    // 稳态时指令会在 -0.2 附近微调，若无滞后，这个灯会闪个不停。
    const bool braking =
        engaged_ ? accel_ < -params_.releaseDecel : accel_ < -params_.engageDecel;
    if (braking && !engaged_) {
        ++interventions_;
    }
    engaged_ = braking;
    return braking;
}

}  // namespace adasim::app
