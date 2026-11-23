#include "application/ExperimentRunner.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "domain/model/ForwardGap.h"
#include "domain/simulation/ActorBehavior.h"

namespace adasim::app {
namespace {

constexpr double kLateralTolerance = 2.5;   // m，判断是否"同车道前方"

/// 确定性伪高斯。
///
/// 用 (seed, step) 直接算出噪声值，而不是维护一个 RNG 对象：后者会让
/// "跑第 300 帧时优化器恰好问过多少次"影响到结果，同一组参数重跑就对不上。
/// 由 (种子, 步号) 唯一决定的值，天然满足可复现。
///
/// 实现取四个均匀分布之和（Irwin–Hall），再按 sqrt(3) 归一化到单位方差。
/// 中心极限已经够用，不必为此引入正态分布的反函数。
double pseudoGauss(std::uint32_t seed, std::uint64_t step) {
    auto uniform = [seed, step](std::uint64_t salt) {
        std::uint64_t x = step * 6364136223846793005ULL + seed * 1442695040888963407ULL + salt;
        x ^= x >> 33;
        x *= 0xff51afd7ed558ccdULL;
        x ^= x >> 33;
        return static_cast<double>(x & 0xFFFFFFu) / 16777216.0;
    };
    const double sum = uniform(1) + uniform(2) + uniform(3) + uniform(4);
    return (sum - 2.0) * 1.7320508075688772;
}

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
                                        const EvaluationConfig& cfg) const {
    ExperimentMetrics metrics;
    if (cfg.dt <= 0.0 || cfg.durationSeconds <= 0.0) {
        return metrics;
    }

    const double begin = clock_.monotonicSeconds();
    const std::uint64_t totalSteps =
        static_cast<std::uint64_t>(std::ceil(cfg.durationSeconds / cfg.dt));

    domain::WorldState state = initial;
    const double startX = state.ego.x;
    const double startY = state.ego.y;
    const double cruise = initial.egoSpeed;

    domain::ControlCommand cmd;

    for (std::uint64_t i = 0; i < totalSteps; ++i) {
        if (cfg.closedLoop) {
            // 控制看"本车道最近的车"（决定该不该刹），指标看"TTC 最小的车"
            // （决定有多危险）。两者口径不同是有意的，见 ForwardGap.h。
            const domain::ForwardGap gap =
                domain::nearestForwardTarget(state, kLateralTolerance);

            double gapObs = gap.bumperGap;
            double leadSpeedObs = state.egoSpeed - gap.closingSpeed;
            if (cfg.noise > 0.0) {
                gapObs += cfg.noise * pseudoGauss(cfg.seed, i * 2 + 1);
                // 速度观测按距离噪声的一半扰动：测距误差会经差分放大成速度误差，
                // 这里简化处理，只保证"距离噪声越大、决策越抖"这个方向正确。
                leadSpeedObs += 0.5 * cfg.noise * pseudoGauss(cfg.seed + 7919, i * 2 + 2);
            }

            cmd.steer = 0.0;
            cmd.targetAccel = domain::idmAccelFor(state.egoSpeed, cruise, gap.valid,
                                                  gapObs, leadSpeedObs);
        } else {
            cmd.steer = 0.0;
            cmd.targetAccel = 0.0;
        }

        state = physics_.step(state, cfg.dt, cmd);
        ++metrics.steps;

        const double ttc = timeToCollision(state);
        metrics.minTtc = std::min(metrics.minTtc, ttc);

        const domain::ForwardGap gapNow =
            domain::nearestForwardTarget(state, kLateralTolerance);
        if (gapNow.valid) {
            metrics.minGap = std::min(metrics.minGap, gapNow.bumperGap);
        }
        metrics.maxAbsAccel = std::max(metrics.maxAbsAccel, std::abs(state.egoAccel));
        if (metrics.minTtc <= 0.0) {
            metrics.collided = true;
            // 撞击时的接近速度。碰撞后立刻终止仿真（省预算），
            // 所以"撞得多狠"只能在这一帧取，晚了就没了。
            metrics.impactSpeed = std::max(0.0, gapNow.closingSpeed);
            break;
        }
    }

    metrics.travelled = std::hypot(state.ego.x - startX, state.ego.y - startY);
    metrics.wallSeconds = clock_.monotonicSeconds() - begin;
    return metrics;
}

ExperimentMetrics ExperimentRunner::run(const domain::WorldState& initial,
                                        double durationSeconds,
                                        double dt) const {
    EvaluationConfig cfg;
    cfg.durationSeconds = durationSeconds;
    cfg.dt = dt;
    cfg.closedLoop = false;   // 旧口径：滑行
    return run(initial, cfg);
}

}  // namespace adasim::app
