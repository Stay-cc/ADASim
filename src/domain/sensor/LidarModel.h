#pragma once

#include <cmath>
#include <cstddef>

#include "domain/model/WorldSnapshot.h"

namespace adasim::domain {

struct LidarParams {
    double range{50.0};        ///< 有效量程，m
    double pointsPerMeter{1.5};///< 轮廓采样密度
    float minIntensity{0.15f};
    /// 点云发布频率，Hz。与物理步长解耦是必须的：物理按 50 Hz 推进，
    /// 但雷达是 10 Hz 的传感器。早期两者同频，2 车 40 点时看不出问题，
    /// 十万点时就是每秒 4.3 万个 UDP 分片（约 51 MB/s），链路直接被灌死。
    double rateHz{10.0};
};

/// 简化的激光雷达模型：取障碍物包围矩形的轮廓点，按量程衰减强度。
///
/// 不是对射线求交，而是对轮廓做等距采样。这个近似在 2D 鸟瞰视图下足够：
/// 点云的用途是给感知算法提供可聚类的观测，而非复现真实反射特性。
/// 十万点量级的压测通过提高 pointsPerMeter 与障碍物数量构造。
struct LidarModel {
    static void sample(const WorldState& state, const LidarParams& params, WorldSnapshot& out) {
        out.clearPoints();
        out.state = state;

        const double spacing = 1.0 / (params.pointsPerMeter > 0.0 ? params.pointsPerMeter : 1.0);
        const double rangeSq = params.range * params.range;

        std::size_t estimate = 0;
        for (const auto& e : state.obstacles) {
            estimate += static_cast<std::size_t>(2.0 * (e.length + e.width) / spacing) + 8;
        }
        for (const auto& e : state.ghosts) {
            estimate += static_cast<std::size_t>(2.0 * (e.length + e.width) / spacing) + 8;
        }
        out.px.reserve(estimate);
        out.py.reserve(estimate);
        out.intensity.reserve(estimate);

        for (const auto& e : state.obstacles) {
            emitOutline(state, e, params, spacing, rangeSq, out);
        }
        // 注入目标走同一条轮廓采样：它要看起来和真实障碍物的点云一模一样，
        // 否则算法一眼就能从点的疏密/强度上区分真假，这种"欺骗"没有意义。
        for (const auto& e : state.ghosts) {
            emitOutline(state, e, params, spacing, rangeSq, out);
        }
    }

private:
    static void emitOutline(const WorldState& state, const EntityState& e,
                            const LidarParams& params, double spacing, double rangeSq,
                            WorldSnapshot& out) {
        const double c = std::cos(e.pose.theta);
        const double s = std::sin(e.pose.theta);
        const double hl = e.length * 0.5;
        const double hw = e.width * 0.5;

        // 车体坐标系下的四个角点，逆时针
        const double localX[4] = {hl, -hl, -hl, hl};
        const double localY[4] = {hw, hw, -hw, -hw};

        for (int edge = 0; edge < 4; ++edge) {
            const int next = (edge + 1) % 4;
            const double x0 = localX[edge];
            const double y0 = localY[edge];
            const double dx = localX[next] - x0;
            const double dy = localY[next] - y0;
            const double len = std::hypot(dx, dy);
            const int samples = static_cast<int>(len / spacing);
            for (int i = 0; i <= samples; ++i) {
                const double t = (samples == 0) ? 0.0 : (static_cast<double>(i) / samples);
                const double lx = x0 + dx * t;
                const double ly = y0 + dy * t;

                const double wx = e.pose.x + c * lx - s * ly;
                const double wy = e.pose.y + s * lx + c * ly;

                const double ddx = wx - state.ego.x;
                const double ddy = wy - state.ego.y;
                const double d2 = ddx * ddx + ddy * ddy;
                if (d2 > rangeSq) {
                    continue;
                }
                const float k = 1.0f - static_cast<float>(std::sqrt(d2) / params.range);
                out.px.push_back(static_cast<float>(wx));
                out.py.push_back(static_cast<float>(wy));
                out.intensity.push_back(k < params.minIntensity ? params.minIntensity : k);
            }
        }
    }
};

}  // namespace adasim::domain
