#include "application/ExperimentRunner.h"

#include <algorithm>
#include <cmath>

#include "domain/model/ForwardGap.h"

namespace adasim::app {
namespace {

constexpr double kLateralTolerance = 2.5;   // m，判断是否"同车道前方"

}  // namespace

ExperimentRunner::ExperimentRunner(domain::PhysicsEngine& physics, domain::IClock& clock)
    : physics_(physics), clock_(clock) {}

double ExperimentRunner::timeToCollision(const domain::WorldState& state) {
    // 走廊几何与 TTC 口径都在领域层（ForwardGap），这里只取聚合结果。
    // 指标取"TTC 最小"而不是"纵向最近"：一个更远但快得多的车可能更危险。
    return domain::criticalForwardTarget(state, kLateralTolerance).ttc;
}

bool ExperimentRunner::hasCollision(const domain::WorldState& state) {
    return timeToCollision(state) <= 0.0;
}

ExperimentMetrics ExperimentRunner::run(const domain::WorldState& initial,
                                        double durationSeconds,
                                        double dt) const {
    ExperimentMetrics metrics;
    if (dt <= 0.0 || durationSeconds <= 0.0) {
        return metrics;
    }

    const double begin = clock_.monotonicSeconds();
    const std::uint64_t totalSteps =
        static_cast<std::uint64_t>(std::ceil(durationSeconds / dt));

    domain::WorldState state = initial;
    const double startX = state.ego.x;
    const double startY = state.ego.y;

    domain::ControlCommand coast;
    coast.steer = 0.0;
    coast.targetAccel = 0.0;

    for (std::uint64_t i = 0; i < totalSteps; ++i) {
        state = physics_.step(state, dt, coast);
        ++metrics.steps;

        metrics.minTtc = std::min(metrics.minTtc, timeToCollision(state));
        metrics.maxAbsAccel = std::max(metrics.maxAbsAccel, std::abs(state.egoAccel));
        if (metrics.minTtc <= 0.0) {
            metrics.collided = true;
            break;  // 碰撞后继续跑没有意义，省下仿真预算
        }
    }

    metrics.travelled = std::hypot(state.ego.x - startX, state.ego.y - startY);
    metrics.wallSeconds = clock_.monotonicSeconds() - begin;
    return metrics;
}

}  // namespace adasim::app
