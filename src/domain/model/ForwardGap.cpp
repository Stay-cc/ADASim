#include "domain/model/ForwardGap.h"

#include <cmath>

namespace adasim::domain {
namespace {

constexpr double kNoTtc = 1e9;
constexpr double kMinClosingSpeed = 0.1;  // m/s，低于此值不算接近

/// 走廊内每个前方目标算一次几何量，两个查询函数只是聚合方式不同。
/// 模板而非 std::function：这是每帧都走的路径，不该为回调付间接调用。
template <typename Fn>
void forEachForward(const WorldState& state, double lateralTolerance, Fn&& fn) {
    const double c = std::cos(state.ego.theta);
    const double s = std::sin(state.ego.theta);

    for (const auto& e : state.obstacles) {
        const double rx = e.pose.x - state.ego.x;
        const double ry = e.pose.y - state.ego.y;
        const double longitudinal = rx * c + ry * s;   // 沿自车航向
        const double lateral = -rx * s + ry * c;       // 垂直航向

        if (longitudinal <= 0.0 || std::abs(lateral) > lateralTolerance) {
            continue;
        }

        ForwardGap g;
        g.valid = true;
        g.id = e.id;
        g.centerGap = longitudinal;
        g.bumperGap = longitudinal - 0.5 * (kEgoBodyLength + e.length);
        // 前车斜着走时，只有速度在自车航向上的投影真正在缩短车距
        g.closingSpeed =
            state.egoSpeed - e.speed * std::cos(e.pose.theta - state.ego.theta);
        g.ttc = g.closingSpeed > kMinClosingSpeed
                    ? std::max(0.0, (g.centerGap - kTtcGapOffset) / g.closingSpeed)
                    : kNoTtc;

        fn(g);
    }
}

}  // namespace

ForwardGap nearestForwardTarget(const WorldState& state, double lateralTolerance) {
    ForwardGap best;
    double bestCenterGap = 1e18;
    forEachForward(state, lateralTolerance, [&](const ForwardGap& g) {
        if (g.centerGap < bestCenterGap) {
            bestCenterGap = g.centerGap;
            best = g;
        }
    });
    return best;
}

ForwardGap criticalForwardTarget(const WorldState& state, double lateralTolerance) {
    ForwardGap best;
    forEachForward(state, lateralTolerance, [&](const ForwardGap& g) {
        if (g.ttc < best.ttc) {
            best = g;
        }
    });
    return best;
}

}  // namespace adasim::domain
