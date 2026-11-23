#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "domain/model/Pose.h"

namespace adasim::domain {

enum class BehaviorKind : std::uint8_t {
    Constant = 0,    ///< 匀速
    CarFollowing,    ///< 跟驰（IDM）
    CutIn,           ///< 切入（到达触发时刻后横向位移）
};

struct ActorSpec {
    std::string type{"car"};
    Pose pose{};
    double speed{0.0};
    BehaviorKind behavior{BehaviorKind::Constant};
    double triggerSeconds{0.0};
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
    SensorSpec sensor{};
    std::vector<ActorSpec> actors;
    std::vector<ParameterSpec> parameters;
};

}  // namespace adasim::domain
