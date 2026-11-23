#include "domain/simulation/ActorBehavior.h"

#include <algorithm>
#include <cmath>

namespace adasim::domain {
namespace {

constexpr double kPi = 3.14159265358979323846;

/// 判定"同一车道"的横向容差。取 1.5 m 而不是半个车道宽（1.75 m）：
/// 切入过半时那辆车已经在两车道之间，若按 1.75 m 判，它会在临界点上
/// 反复进出前车集合，跟驰加速度随之跳变。
constexpr double kSameLaneTolerance = 1.5;

/// 跟驰只看 80 m 以内：再远的车对加速度的影响已小于 0.01 m/s^2，
/// 却会让 792 车的压测场景多出 O(n^2) 的比较量。
constexpr double kLeadSearchRange = 80.0;

// IDM 参数取乘用车常用值（Treiber & Kesting 的建议区间）。
constexpr double kIdmAccelMax = 1.5;
constexpr double kIdmDecelComfort = 2.0;
constexpr double kIdmStandstill = 2.0;
constexpr double kIdmHeadway = 1.2;

struct LeadInfo {
    bool found{false};
    double bumperGap{0.0};
    double speed{0.0};  ///< 前车沿本车航向的速度分量
};

LeadInfo findLead(const WorldState& state, const EntityState& self) {
    LeadInfo best;
    double bestGap = kLeadSearchRange;

    const double c = std::cos(self.pose.theta);
    const double s = std::sin(self.pose.theta);

    for (const auto& other : state.obstacles) {
        if (other.id == self.id) {
            continue;
        }
        const double dx = other.pose.x - self.pose.x;
        const double dy = other.pose.y - self.pose.y;
        const double longitudinal = dx * c + dy * s;
        const double lateral = -dx * s + dy * c;

        if (std::abs(lateral) > kSameLaneTolerance) {
            continue;
        }
        // 净距用中心距减两车半长：车长不同时（轿车 vs 卡车）中心距会给出乐观的余量。
        const double gap = longitudinal - 0.5 * (self.length + other.length);
        if (gap < -1.0 || gap > bestGap) {
            continue;
        }
        bestGap = gap;
        best.found = true;
        best.bumperGap = gap;
        best.speed = other.speed * std::cos(other.pose.theta - self.pose.theta);
    }
    return best;
}

/// 智能驾驶员模型（IDM）。
double idmAccel(const EntityState& self, const LeadInfo& lead) {
    const double v = std::max(0.0, self.speed);
    const double desired = self.behavior.desiredSpeed > 0.0 ? self.behavior.desiredSpeed : v;
    const double v0 = std::max(0.5, desired);

    double accel = kIdmAccelMax * (1.0 - std::pow(v / v0, 4.0));
    if (!lead.found) {
        return std::clamp(accel, -kIdmAccelMax, kIdmAccelMax);
    }

    const double closing = v - lead.speed;
    const double sStar = kIdmStandstill +
                         std::max(0.0, v * kIdmHeadway +
                                           v * closing / (2.0 * std::sqrt(kIdmAccelMax *
                                                                          kIdmDecelComfort)));
    const double ratio = sStar / std::max(lead.bumperGap, 0.3);
    accel -= kIdmAccelMax * ratio * ratio;
    return std::clamp(accel, -kIdmDecelComfort * 3.0, kIdmAccelMax);
}

void advanceStraight(EntityState& e, double dt) {
    e.pose.theta += e.yawRate * dt;
    e.pose.x += e.speed * std::cos(e.pose.theta) * dt;
    e.pose.y += e.speed * std::sin(e.pose.theta) * dt;
}

void advanceFollowing(const WorldState& in, EntityState& e, double dt) {
    const LeadInfo lead = findLead(in, e);
    const double a = idmAccel(e, lead);
    e.speed = std::max(0.0, e.speed + a * dt);
    advanceStraight(e, dt);
}

void advanceCutIn(EntityState& e, double dt, double simTime) {
    const double span = e.behavior.targetY - e.behavior.originY;
    const double duration = std::max(0.2, e.behavior.laneChangeSeconds);
    const double elapsed = simTime - e.behavior.triggerSeconds;

    // 未到触发时刻，或目标车道与当前车道重合：按匀速直行处理。
    if (std::abs(span) < 1e-9 || elapsed <= 0.0) {
        advanceStraight(e, dt);
        return;
    }

    const double progress = std::clamp(elapsed / duration, 0.0, 1.0);
    const double ratio = cutInLateralRatio(progress);

    // 横向位置取解析解而不是积分：积分每步都会累积截断误差，
    // 复现时只要步数一致才不会漂；解析解与步长无关，重推演必然一致。
    e.pose.y = e.behavior.originY + span * ratio;

    // 横向速度 → 航向角。用 asin 而不是直接给个常数角速度：
    // 这样点云里车身是"斜着切进来"的，且端点横向速度为零（航向连续回到 0）。
    const double lateralSpeed = span * 0.5 * (kPi / duration) * std::sin(kPi * progress);
    const double speed = std::max(std::abs(e.speed), 0.5);
    e.pose.theta = std::asin(std::clamp(lateralSpeed / speed, -0.6, 0.6));

    // 制动从横向过半开始：这是"别车后立刻踩刹车"的时序，
    // 若等平移完全结束再刹，两车横向已无重叠，威胁就不存在了。
    if (progress >= 0.5 && e.behavior.brakeAccel > 0.0) {
        e.speed = std::max(e.behavior.speedFloor, e.speed - e.behavior.brakeAccel * dt);
    }

    e.pose.x += e.speed * std::cos(e.pose.theta) * dt;
}

}  // namespace

double cutInLateralRatio(double progress) {
    const double p = std::clamp(progress, 0.0, 1.0);
    return 0.5 * (1.0 - std::cos(kPi * p));
}

double idmAccelFor(const WorldState& state, const EntityState& self) {
    return idmAccel(self, findLead(state, self));
}

void advanceActor(const WorldState& in, const EntityState& before, EntityState& after,
                  double dt) {
    after = before;
    switch (before.behavior.kind) {
        case BehaviorKind::Constant:
            advanceStraight(after, dt);
            break;
        case BehaviorKind::CarFollowing:
            advanceFollowing(in, after, dt);
            break;
        case BehaviorKind::CutIn:
            advanceCutIn(after, dt, in.simTime + dt);
            break;
    }
    // 加速度由速度差反推，避免行为分支各自维护一份而与实际运动不符。
    after.accel = dt > 0.0 ? (after.speed - before.speed) / dt : 0.0;
}

void advanceActors(const WorldState& in, WorldState& out, double dt) {
    const std::size_t n = std::min(in.obstacles.size(), out.obstacles.size());
    for (std::size_t i = 0; i < n; ++i) {
        advanceActor(in, in.obstacles[i], out.obstacles[i], dt);
    }
}

}  // namespace adasim::domain
