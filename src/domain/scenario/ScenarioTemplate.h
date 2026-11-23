#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "domain/model/Behavior.h"
#include "domain/model/Pose.h"

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
    SensorSpec sensor{};
    std::vector<ActorSpec> actors;
    std::vector<ParameterSpec> parameters;
};

}  // namespace adasim::domain
