#pragma once

#include <cstdint>

#include "domain/model/WorldState.h"

namespace adasim::domain {

/// 本车道前方最近目标的几何关系。
///
/// 这个结构只回答"前面那辆车离我多远、以多快逼近、还有多久撞上"，不含任何决策。
/// 之所以抽出来，是因为有两个地方要问同一个问题：
///   - ExperimentRunner 统计最小 TTC（实验指标口径）
///   - StandbyGuard 在算法进程缺席时判断该不该踩刹车（控制口径）
/// 各写一份的话，走廊容差或净距口径迟早会分叉，指标与兜底行为就对不上了。
struct ForwardGap {
    bool valid{false};
    std::uint32_t id{0};
    double centerGap{0.0};     ///< 纵向中心距，m
    double bumperGap{0.0};     ///< 自车车头到前车车尾的净距，m，可为负（已接触）
    double closingSpeed{0.0};  ///< 接近速度，m/s；<=0 表示正在拉开距离
    double ttc{1e9};           ///< 碰撞时间，s；不接近时为 1e9
};

/// 净距需要自车车长，而 WorldState 不带自车尺寸（物理层只积分位姿）。
/// 平台默认乘用车 4.5 m，与场景默认值、LidarModel 采样一致。
inline constexpr double kEgoBodyLength = 4.5;

/// TTC 口径：中心距减去该值作为"有效接近距离"，与 3.5 节的指标定义一致。
/// 沿用历史口径而未改成净距，是为了让新旧实验数据可以直接对比。
inline constexpr double kTtcGapOffset = 3.0;

/// 纵向最近的前方目标（控制口径：要拿它的净距算制动距离）。
[[nodiscard]] ForwardGap nearestForwardTarget(const WorldState& state,
                                              double lateralTolerance = 2.5);

/// TTC 最小的前方目标（指标口径：一个更远但更快逼近的车可能更危险）。
[[nodiscard]] ForwardGap criticalForwardTarget(const WorldState& state,
                                               double lateralTolerance = 2.5);

}  // namespace adasim::domain
