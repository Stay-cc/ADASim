#pragma once

#include <cstdint>

#include "domain/model/WorldState.h"
#include "domain/port/IClock.h"
#include "domain/simulation/PhysicsEngine.h"

namespace adasim::app {

/// 单次离线仿真的结果指标。失效搜索把它当作评价函数取值，
/// 实验章节的量化对比也直接取这里的字段。
struct ExperimentMetrics {
    std::uint64_t steps{0};
    bool collided{false};
    double minTtc{1e9};        ///< 最小碰撞时间，s；未接近过则保持初值
    double maxAbsAccel{0.0};   ///< 最大纵向加速度绝对值，m/s^2
    double travelled{0.0};     ///< 自车行驶距离，m
    double wallSeconds{0.0};   ///< 真实耗时，用于评估仿真吞吐
};

/// 离线批量运行器：跑完一个场景、取回指标，不涉及界面与实时节拍。
/// AI 进程的失效搜索通过它执行"给定参数 → 得到指标"的原子操作。
class ExperimentRunner {
public:
    ExperimentRunner(domain::PhysicsEngine& physics, domain::IClock& clock);

    /// 以零控制指令（滑行）跑完 durationSeconds，返回指标。
    [[nodiscard]] ExperimentMetrics run(const domain::WorldState& initial,
                                        double durationSeconds,
                                        double dt) const;

    /// 计算当前状态下的最小碰撞时间；无接近目标时返回初值 1e9。
    [[nodiscard]] static double timeToCollision(const domain::WorldState& state);

    [[nodiscard]] static bool hasCollision(const domain::WorldState& state);

private:
    domain::PhysicsEngine& physics_;
    domain::IClock& clock_;
};

}  // namespace adasim::app
