#pragma once

#include <cmath>
#include <vector>

namespace adasim::domain {

/// 道路的一个分段：从里程 startS 到 endS 这一段，有自己独立的曲率与车道布局。
///
/// 分段道路是"超大地图"（沙盒）的地基：一张地图不再是单一曲率的圆弧，
/// 而是直道、不同弯向的弯道、多车道路段按里程拼接成的折线。每段沿道路
/// 中心线的里程用 s 度量，自车每帧累计里程后查表定位到当前段。
///
/// 曲率约定与单曲率场景一致：正值左弯、负值右弯、0 为直道。段的曲率是
/// 常数（圆弧），相邻段在交界处曲率跳变——这简化了参考路径的生成，
/// 代价是交界处横向误差会有一次阶跃，这在"超大地图"场景下可接受。
struct RoadSegment {
    double startS{0.0};        ///< 段起点里程，m
    double endS{0.0};          ///< 段终点里程，m（必须 > startS）
    double curvature{0.0};     ///< 1/m，该段道路中心线曲率
    int laneCount{2};          ///< 该段车道数
    double laneWidth{3.5};     ///< 该段车道宽，m
};

/// 按里程查路段。里程落在某段 [startS, endS) 内返回该段指针，否则返回 null。
/// 道路定义为空或 s 超出全部段时也返回 null，调用方据此回退到单曲率字段。
inline const RoadSegment* segmentAt(const std::vector<RoadSegment>& segs, double s) {
    for (const auto& seg : segs) {
        if (s >= seg.startS && s < seg.endS) {
            return &seg;
        }
    }
    return nullptr;
}

/// 道路中心线上一点的世界坐标与切线角。
///
/// 分段道路的世界几何由"路径积分"得到：从里程 0 的起点（世界原点，朝向 +x）
/// 出发，逐段累加。直道沿切线直线延伸；弯道是圆弧，半径 R = 1/κ、弧长 Δs、
/// 转过角度 Δθ = κ·Δs。交点处切线连续（C1），曲率跳变（C0 于曲率）。
struct RoadPose {
    double x{0.0};      ///< 中心线世界 x
    double y{0.0};      ///< 中心线世界 y
    double theta{0.0};  ///< 中心线切线角（世界系，rad）
};

/// 求里程 s 处道路中心线的世界位姿。segs 必须按 startS 升序且连续拼接。
/// s 超出范围时，按最后一段的末端切线外推（直道外推），保证"开出地图"也稳定。
inline RoadPose roadPoseAt(const std::vector<RoadSegment>& segs, double s) {
    RoadPose out;  // 起点：世界原点，朝 +x
    if (segs.empty()) {
        out.x = s;
        return out;
    }

    double cursor = 0.0;  // 已走过的里程
    for (const auto& seg : segs) {
        const double segStart = seg.startS;
        const double segEnd = seg.endS;
        const double len = segEnd - segStart;
        if (len <= 0.0) {
            continue;
        }
        // 目标里程落在本段内：在本段内部再前进 (s - cursor)
        if (s <= segEnd) {
            const double run = s - cursor;
            const double kappa = seg.curvature;
            if (std::abs(kappa) < 1e-9) {
                out.x += run * std::cos(out.theta);
                out.y += run * std::sin(out.theta);
            } else {
                const double R = 1.0 / kappa;
                const double dTheta = kappa * run;
                // 圆心在起点沿法向（左为 +y 法向）偏移 R
                const double cx = out.x - R * std::sin(out.theta);
                const double cy = out.y + R * std::cos(out.theta);
                // 从起点到圆心的向量角，转 dTheta
                const double a0 = std::atan2(out.y - cy, out.x - cx);
                const double a1 = a0 + dTheta;
                out.x = cx + R * std::cos(a1);
                out.y = cy + R * std::sin(a1);
                out.theta += dTheta;
            }
            return out;
        }
        // 整段都要走完
        const double kappa = seg.curvature;
        if (std::abs(kappa) < 1e-9) {
            out.x += len * std::cos(out.theta);
            out.y += len * std::sin(out.theta);
        } else {
            const double R = 1.0 / kappa;
            const double dTheta = kappa * len;
            const double cx = out.x - R * std::sin(out.theta);
            const double cy = out.y + R * std::cos(out.theta);
            const double a0 = std::atan2(out.y - cy, out.x - cx);
            const double a1 = a0 + dTheta;
            out.x = cx + R * std::cos(a1);
            out.y = cy + R * std::sin(a1);
            out.theta += dTheta;
        }
        cursor = segEnd;
    }
    // 超出全部段：沿末端切线外推
    const double run = s - cursor;
    out.x += run * std::cos(out.theta);
    out.y += run * std::sin(out.theta);
    return out;
}

/// 道路中心线总长。
inline double roadLength(const std::vector<RoadSegment>& segs) {
    if (segs.empty()) {
        return 0.0;
    }
    return segs.back().endS;
}

}  // namespace adasim::domain
