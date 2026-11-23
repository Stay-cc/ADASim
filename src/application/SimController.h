#pragma once

#include <memory>

#include "application/StandbyGuard.h"
#include "domain/model/WorldSnapshot.h"
#include "domain/port/IClock.h"
#include "domain/port/ITransportPort.h"
#include "domain/sensor/LidarModel.h"
#include "domain/simulation/PhysicsEngine.h"
#include "domain/timeline/TimelineBuffer.h"

namespace adasim::app {

/// 仿真控制门面：表现层唯一需要认识的编排对象。
///
/// 它持有的是端口引用与领域对象引用，不认识任何具体实现。
/// 播放/暂停/倍速/单步/回溯都在这里编排，但物理怎么算在 L3，
/// 报文怎么发在 L4，本类只决定"什么时候调用谁"。
class SimController {
public:
    SimController(domain::PhysicsEngine& physics,
                  domain::TimelineBuffer& buffer,
                  domain::ITransportPort& transport,
                  domain::IClock& clock);

    void reset(const domain::WorldState& initial);
    void start();
    void pause();
    void toggle();
    [[nodiscard]] bool running() const noexcept { return running_; }

    /// 由界面定时器驱动。按真实经过时间累积，换算成整数个物理步进，
    /// 单次最多补 8 步，避免卡顿后出现"追帧风暴"。
    bool stepIfDue(double nowMonotonicSeconds);

    /// 无条件推进一步，供单步调试使用。
    void stepOnce();

    /// 回溯到指定帧。命中快照直接取用，否则从最近快照重推演。
    /// 成功后写指针回退，后续仿真将覆盖该帧之后的记录。
    bool seek(domain::FrameId target);

    void setSpeedFactor(double factor);
    [[nodiscard]] double speedFactor() const noexcept { return speedFactor_; }

    [[nodiscard]] const domain::WorldState& current() const noexcept { return current_; }
    [[nodiscard]] domain::FrameId latestFrame() const noexcept { return current_.frameId; }

    /// 构造（或复用）只读快照，供渲染端口消费。
    std::shared_ptr<const domain::WorldSnapshot> snapshot();

    [[nodiscard]] double fixedStep() const noexcept { return fixedStep_; }
    void setFixedStep(double dt);

    /// 断线兜底是否正在制动。仅在没有算法进程在线时可能为真，
    /// 界面拿它提示用户"接管缺失、底座在自救"，避免被误读成算法在刹车。
    [[nodiscard]] bool standbyGuardActive() const noexcept { return standbyGuard_.engaged(); }
    [[nodiscard]] std::uint64_t standbyInterventions() const noexcept {
        return standbyGuard_.interventions();
    }

private:
    void publish();
    void buildSnapshotCache();

    domain::PhysicsEngine& physics_;
    domain::TimelineBuffer& buffer_;
    domain::ITransportPort& transport_;
    domain::IClock& clock_;

    domain::WorldState current_{};
    domain::ControlCommand lastCommand_{};
    std::shared_ptr<domain::WorldSnapshot> snapshot_;
    StandbyGuard standbyGuard_;

    domain::LidarParams lidar_{};
    double fixedStep_{0.02};
    double speedFactor_{1.0};
    double accumulator_{0.0};
    double lastWall_{0.0};
    bool running_{false};
};

}  // namespace adasim::app
