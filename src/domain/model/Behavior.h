#pragma once

#include <cstdint>

namespace adasim::domain {

/// 交通参与者的行为类型。
///
/// 放在 model 层而不是 scenario 层：行为参数要随 WorldState 一起进时间轴快照，
/// 否则回溯到某一帧重推演时，代码无从知道那辆车当时处在哪个阶段，
/// 重推演结果就不可能和直跑逐位一致。
enum class BehaviorKind : std::uint8_t {
    Constant = 0,    ///< 匀速直行
    CarFollowing,    ///< 跟驰：按 IDM 追击同车道前方最近的车
    CutIn,           ///< 切入：到触发时刻横向平移至目标车道，过半后开始制动
    Brake,           ///< 制动：到触发时刻起以 brakeAccel 减速至 speedFloor
};

/// 行为参数。全部为绝对值，不含任何"已走了几步"的隐藏计数器——
/// 这让推进函数成为 (状态, simTime) 的纯函数，重推演才可复现。
struct BehaviorState {
    BehaviorKind kind{BehaviorKind::Constant};

    double triggerSeconds{0.0};     ///< 绝对仿真时刻；0 表示一开始就生效
    double originY{0.0};            ///< 初始横向坐标，切入的起点
    double targetY{0.0};            ///< 切入目标横向坐标（车道中心）
    double laneChangeSeconds{4.0};  ///< 横向平移总时长，s
    double brakeAccel{0.0};         ///< m/s^2，切入过半后的减速度（正值；0 表示不减速）
    double speedFloor{0.0};         ///< m/s，减速到此为止
    double desiredSpeed{0.0};       ///< m/s，跟驰的期望车速（<=0 时取当前车速）
};

}  // namespace adasim::domain
