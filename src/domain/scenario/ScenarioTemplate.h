#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "domain/model/Behavior.h"
#include "domain/model/Pose.h"
#include "domain/model/RoadSegment.h"

namespace adasim::domain {

/// 场景里的交通参与者。behavior 与 triggerSeconds 在这里只是"描述"，
/// 真正参与推进的是世界状态里那份 BehaviorState（由 ScenarioService 实例化）。
struct ActorSpec {
    std::string type{"car"};
    Pose pose{};
    double speed{0.0};
    BehaviorKind behavior{BehaviorKind::Constant};
    double triggerSeconds{0.0};

    double targetY{0.0};            ///< 切入目标横向坐标；仅在 explicitly 为真时生效
    bool targetYExplicit{false};
    double laneChangeSeconds{4.0};  ///< 横向平移总时长
    double brakeAccel{0.0};         ///< 切入过半后的减速度，m/s^2
    double speedFloor{0.0};         ///< 减速到此为止，m/s
};

/// 可搜索参数的定义。语言解析与失效搜索共用这一份描述：
/// 前者产出参数名与取值范围，后者在该范围内采样。
struct ParameterSpec {
    std::string name;
    double min{0.0};
    double max{1.0};
    double step{0.1};
};

/// 点云传感器的采样参数。
///
/// 放进场景模板而不是写死在控制器里，是因为点云规模直接决定渲染、网络
/// 与感知三处的开销：同一个场景换一种采样密度就是另一个量级的工作负载。
/// 压测场景若不能自己声明"我这帧有多少点"，结论就无法复现。
struct SensorSpec {
    double lidarRange{50.0};           ///< m，有效量程
    double lidarPointsPerMeter{1.5};   ///< 轮廓采样密度，点/米
};

/// 参数化场景模板。既是自然语言解析的输出，也是失效搜索的输入。
struct ScenarioTemplate {
    std::string id;
    std::string name;
    int laneCount{2};
    double laneWidth{3.5};      ///< m
    double curvature{0.0};      ///< 1/m，0 为直道
    /// 分段道路定义。为空时沿用上面的单曲率字段（旧场景语义不变）；
    /// 非空时按里程查段，curvature 字段仅作首段曲率的兜底值。
    std::vector<RoadSegment> roadSegments;
    double friction{0.9};       ///< 路面附着系数
    double visibility{1.0};     ///< 能见度系数，1 为晴好
    double durationSeconds{30.0};
    /// 自车初速，m/s。写进模板而不是只放在 parameters 里，是因为它既是
    /// 场景的物理属性（决定几何与时序），也是失效搜索的搜索变量之一。
    /// 早先只能从 `parameters` 里找一个叫 ego_speed 的项取 min，那让
    /// "搜索下界"和"实际取值"绑成同一个数，改一个必伤另一个。
    double egoSpeed{8.0};
    /// egoSpeed 是否由文件显式给出。未给出时回退到旧的 parameters 约定，
    /// 保证既有场景文件的语义不变。
    bool egoSpeedExplicit{false};
    /// 自车初始位姿，缺省为原点朝 +x。留出这个字段是为了能考"横向偏差
    /// 收敛"：自车若永远从参考线正中起步，直道场景的横向误差恒为 0，
    /// 那个指标就没有任何区分度，测出来的是"没有扰动"而不是"跟踪得好"。
    Pose egoPose{};
    SensorSpec sensor{};
    std::vector<ActorSpec> actors;
    std::vector<ParameterSpec> parameters;

    /// 开放世界（无尽沙盒）：没有预设参与者与结束时刻，交通由用户运行时注入。
    ///
    /// 它只做一件事——让"空场景"成为合法场景。校验里那条"至少需要一个
    /// 交通参与者"是防手写场景漏配的，沙盒恰恰要从零开始，所以按这个标志放行。
    /// 但不可放宽为无条件允许空 actors：那样普通场景写漏 actors 就查不出来了。
    bool openWorld{false};
};

}  // namespace adasim::domain
