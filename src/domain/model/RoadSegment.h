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
///
/// 弯道几何：圆心在起点沿"弯向的法向"偏移 R = 1/|κ|（左弯左法向、右弯右法向），
/// 转角 dtheta = κ·len。半径必须取正——若沿用有符号 R = 1/κ，终点公式
/// cx + R·cos 会把符号带错，右弯路径会跳到错误位置（S 弯交界处跳几百米）。
inline RoadPose roadPoseAt(const std::vector<RoadSegment>& segs, double s) {
    RoadPose out;  // 起点：世界原点，朝 +x
    if (segs.empty()) {
        out.x = s;
        return out;
    }

    // 沿当前切线前进 run 米（直道），或走半径 1/|κ| 的圆弧（弯道）。
    const auto advance = [&out](double run, double kappa) {
        if (std::abs(kappa) < 1e-9) {
            out.x += run * std::cos(out.theta);
            out.y += run * std::sin(out.theta);
            return;
        }
        const double radius = std::abs(1.0 / kappa);  // 正半径
        const double dTheta = kappa * run;            // 有符号转角
        const double sign = kappa > 0.0 ? 1.0 : -1.0; // 左弯用左法向、右弯用右法向
        const double cx = out.x - sign * radius * std::sin(out.theta);
        const double cy = out.y + sign * radius * std::cos(out.theta);
        const double a0 = std::atan2(out.y - cy, out.x - cx);
        const double a1 = a0 + dTheta;
        out.x = cx + radius * std::cos(a1);
        out.y = cy + radius * std::sin(a1);
        out.theta += dTheta;
    };

    double cursor = 0.0;  // 已走过的里程
    for (const auto& seg : segs) {
        const double segEnd = seg.endS;
        const double len = segEnd - seg.startS;
        if (len <= 0.0) {
            continue;
        }
        if (s <= segEnd) {
            advance(s - cursor, seg.curvature);
            return out;
        }
        advance(len, seg.curvature);
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
