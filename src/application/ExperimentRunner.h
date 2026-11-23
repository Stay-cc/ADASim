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
    double minGap{1e9};        ///< 最小净距，m；负值表示已接触
    /// 碰撞瞬间的接近速度，m/s。没有它，所有碰撞样本的严重度都一样——
    /// 因为碰撞后立即终止仿真，穿透深度永远只是"刚接触"那一帧的值，
    /// 搜索器在碰撞区就失去了梯度，只能随机游走。
    double impactSpeed{0.0};
    double wallSeconds{0.0};   ///< 真实耗时，用于评估仿真吞吐
};

/// 一次离线评估的运行参数。
///
/// `closedLoop` 为真时自车按 IDM 自动跟驰，否则沿初始速度滑行。
/// 失效搜索必须用闭环——滑行的自车对任何场景都只有一种反应（撞上去），
/// 目标函数在参数空间里几乎是常数，"搜索"就退化成随机抽样。
struct EvaluationConfig {
    double durationSeconds{20.0};
    double dt{0.02};
    /// 感知噪声：前车距离的观测标准差，m。只污染控制器的输入，
    /// 不污染指标计算——否则噪声会直接算进 TTC，测出来的是"噪声"而不是"系统失效"。
    double noise{0.0};
    /// 噪声的随机种子。默认固定值：同一组参数必须复现同一结果，
    /// 否则贝叶斯优化的 GP 拟合的是噪声本身。
    std::uint32_t seed{1};
    bool closedLoop{true};
};

/// 离线批量运行器：跑完一个场景、取回指标，不涉及界面与实时节拍。
/// AI 进程的失效搜索通过它执行"给定参数 → 得到指标"的原子操作。
class ExperimentRunner {
public:
    ExperimentRunner(domain::PhysicsEngine& physics, domain::IClock& clock);

    /// 按配置跑完，返回指标。
    [[nodiscard]] ExperimentMetrics run(const domain::WorldState& initial,
                                        const EvaluationConfig& cfg) const;

    /// 以零控制指令（滑行）跑完 durationSeconds，返回指标。
    /// 保留此重载是为了让既有实验脚本与文档里的数字保持可比。
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
