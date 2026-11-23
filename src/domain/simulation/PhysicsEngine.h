#pragma once

#include "domain/model/ControlCommand.h"
#include "domain/model/VehicleParams.h"
#include "domain/model/WorldState.h"

namespace adasim::domain {

/// 物理推演引擎。纯计算、无副作用：给定状态、步长与控制量，返回下一帧状态。
///
/// 自车采用运动学自行车模型，四阶龙格—库塔积分（见 5.3.4）。
/// 障碍物按各自速度沿航向做匀速推进，行为层（跟驰/切入）在后续版本接入。
/// 无状态设计使得回溯重推演可以直接复用同一套代码路径。
class PhysicsEngine {
public:
    explicit PhysicsEngine(VehicleParams params = {});

    [[nodiscard]] WorldState step(const WorldState& in, double dt,
                                  const ControlCommand& cmd) const;

    [[nodiscard]] const VehicleParams& params() const noexcept { return params_; }

private:
    [[nodiscard]] Pose integrate(const Pose& pose, double speed, double steer, double dt) const;

    VehicleParams params_;
};

}  // namespace adasim::domain
