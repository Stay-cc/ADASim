#pragma once

#include <cstdint>
#include <vector>

#include "domain/model/WorldState.h"

namespace adasim::domain {

/// UDP 通道载荷：高频点云，允许丢帧，因此不做重传也不做确认。
struct PointCloudPacket {
    FrameId frameId{0};
    double simTime{0.0};
    std::vector<float> px;
    std::vector<float> py;
    std::vector<float> intensity;

    [[nodiscard]] std::size_t pointCount() const noexcept { return px.size(); }
};

/// TCP 通道载荷：结构化状态，需要可靠有序。
struct StatePacket {
    FrameId frameId{0};
    double simTime{0.0};
    Pose ego{};
    double egoSpeed{0.0};
    std::vector<EntityState> obstacles;
};

}  // namespace adasim::domain
