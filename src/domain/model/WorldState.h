#pragma once

#include <cstdint>
#include <vector>

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
};

/// 一帧的完整世界状态。这是领域层在各层之间传递的核心值类型。
struct WorldState {
    FrameId frameId{0};
    double simTime{0.0};       ///< 仿真时钟，秒
    Pose ego{};
    double egoSpeed{0.0};      ///< m/s
    double egoAccel{0.0};      ///< m/s^2
    double steer{0.0};         ///< 前轮转角，rad
    std::vector<EntityState> obstacles;
};

}  // namespace adasim::domain
