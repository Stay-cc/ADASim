#pragma once

#include <cstdint>
#include <vector>

#include "domain/model/RoadSegment.h"
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
    double curvature{0.0};   ///< 道路中心线曲率，1/m；算法侧据此生成参考路径
    /// 分段道路定义（超大地图）。为空表示单曲率场景，算法沿用 curvature。
    /// 每帧随状态下发：道路定义很短（几段），开销可忽略，换来算法侧无需
    /// "先收道路再处理状态"的握手顺序。
    std::vector<RoadSegment> roadSegments;
    /// 自车里程，用于算法侧在分段道路上定位当前参考点。
    double egoOdometer{0.0};
    std::vector<EntityState> obstacles;
};

}  // namespace adasim::domain
