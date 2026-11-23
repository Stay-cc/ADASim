#pragma once

#include "domain/model/ControlCommand.h"
#include "domain/model/Packet.h"

namespace adasim::domain {

/// 通信端口。把"双通道、分帧、心跳"这些细节挡在领域层之外，
/// 领域层只看到三件事：发点云、发状态、取指令。
struct ITransportPort {
    virtual ~ITransportPort() = default;

    /// UDP，时效优先，允许丢帧。
    virtual void publishCloud(const PointCloudPacket& packet) = 0;

    /// TCP，可靠性优先。
    virtual void publishState(const StatePacket& packet) = 0;

    /// 非阻塞取一条控制指令；无指令时返回 false。
    [[nodiscard]] virtual bool pollCommand(ControlCommand& out) = 0;

    /// 心跳是否存活。断线时底座应进入保持模式而非继续推演。
    [[nodiscard]] virtual bool linkAlive() const noexcept = 0;
};

}  // namespace adasim::domain
