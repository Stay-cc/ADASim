#pragma once

#include <cstdint>
#include <vector>

#include "domain/model/Behavior.h"
#include "domain/model/Pose.h"
#include "domain/model/Types.h"

namespace adasim::domain {

enum class EntityKind : std::uint8_t {
    Unknown = 0,
    Car,
    Truck,
    Pedestrian,
    Cyclist,
};

/// 交通参与者的动态状态。尺寸字段参与包围盒计算，供点云生成使用。
struct EntityState {
    std::uint32_t id{0};
    EntityKind kind{EntityKind::Car};
    Pose pose{};
    double speed{0.0};    ///< 沿航向的速度，m/s
    double accel{0.0};    ///< 纵向加速度，m/s^2
    double yawRate{0.0};  ///< 航向角速度，rad/s
    double length{4.5};   ///< m
    double width{1.9};    ///< m
    BehaviorState behavior{};  ///< 行为类型与参数，随状态进快照
};

/// 一帧的完整世界状态。这是领域层在各层之间传递的核心值类型。
struct WorldState {
    FrameId frameId{0};
    double simTime{0.0};       ///< 仿真时钟，秒
    Pose ego{};
    double egoSpeed{0.0};      ///< m/s
    double egoAccel{0.0};      ///< m/s^2
    double steer{0.0};         ///< 前轮转角，rad
    /// 路面附着系数。放进状态而不是放在引擎的成员变量里，有两个原因：
    /// 一是它对所有人（自车制动、参与者制动）都是同一个量，该由状态携带；
    /// 二是回溯重推演时它必须跟着帧一起回退，成员变量做不到这一点。
    double friction{0.9};
    /// 道路中心线曲率，1/m，正值表示左弯（圆心在 +y 侧）。
    /// 与 friction 同理，它是"这一帧的道路属性"，必须随快照一起回退，
    /// 否则回溯到直道段时算法侧仍按弯道生成参考路径。
    double curvature{0.0};
    /// 车道数与车道宽，与 friction 同类：都是"这一帧的路"。
    /// 渲染层靠它画出车道线，否则画面上只有一层网格和三块矩形，
    /// 看不出车跑在路上、也看不出切入是从哪条道切过来的。
    int laneCount{2};
    double laneWidth{3.5};
    std::vector<EntityState> obstacles;
    /// 运行时注入的虚拟目标（沙盒模式右键点出来那些）。
    ///
    /// 单列一份而不是塞进 obstacles，是整套"雷达欺骗"的关键：
    /// 它们要被激光雷达扫到（进点云），但**不能**进 StatePacket.obstacles
    /// ——算法若从真值列表里直接读到一个凭空出现的障碍，这次注入就变成了
    /// "给答案"而不是"给观测"。只有走点云感知才能发现它，才谈得上闭环验证。
    /// 它们也不参与物理推进与碰撞：注入的是"雷达看到的东西"，不是刚体。
    std::vector<EntityState> ghosts;
};

}  // namespace adasim::domain
