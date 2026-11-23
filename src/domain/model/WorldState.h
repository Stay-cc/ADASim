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
    std::vector<EntityState> obstacles;
};

}  // namespace adasim::domain
