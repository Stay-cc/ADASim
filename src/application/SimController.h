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

    /// 只读回看：把状态还原到 target，但**不**丢弃其后的历史。
    ///
    /// 行车记录仪的拖拽需要来回看，每次都截断的话往回拖一段之后
    /// 再往右拖就再也没有数据了。截断推迟到真正恢复播放时（start）才发生，
    /// 这时的语义是"从这里另起一支"，而那正是用户按下播放的意图。
    bool previewSeek(domain::FrameId target);

    /// 在此截断历史：丢弃当前帧之后的记录。单步与恢复播放前都要调它，
    /// 否则 append 会落在旧的写指针上，新帧的帧号直接跳到时间轴末尾。
    void truncateHere();

    /// 窗口内最旧的帧号。滑块下限，历史被环形缓冲覆盖后会前移。
    [[nodiscard]] domain::FrameId windowStart() const noexcept { return buffer_.earliest(); }

    /// 最近一次被采纳的控制指令。HUD 用它显示算法输出（加速度/转角/决策）。
    [[nodiscard]] const domain::ControlCommand& lastCommand() const noexcept {
        return lastCommand_;
    }

    /// 注入一个虚拟目标：进点云、不进真值、不参与碰撞。返回其 id。
    std::uint32_t injectGhost(double x, double y, double theta, domain::EntityKind kind);
    void clearGhosts();
    [[nodiscard]] std::size_t ghostCount() const noexcept { return current_.ghosts.size(); }

    void setSpeedFactor(double factor);
    [[nodiscard]] double speedFactor() const noexcept { return speedFactor_; }

    [[nodiscard]] const domain::WorldState& current() const noexcept { return current_; }
    /// 当前所在帧（回看时是历史帧）。
    [[nodiscard]] domain::FrameId latestFrame() const noexcept { return current_.frameId; }
    /// 已录到的最后一帧。回看时它与 latestFrame 不同——
    /// 进度条上限必须取这个，否则往回拖一次，滑块的可拖范围就塌到当前位置了。
    [[nodiscard]] domain::FrameId bufferLatest() const noexcept { return buffer_.latest(); }

    /// 构造（或复用）只读快照，供渲染端口消费。
    std::shared_ptr<const domain::WorldSnapshot> snapshot();

    [[nodiscard]] double fixedStep() const noexcept { return fixedStep_; }
    void setFixedStep(double dt);

    /// 设定点云采样参数。由场景文件的 sensor 块给出，缺省即为 LidarParams 默认值。
    void setLidarParams(const domain::LidarParams& params) { lidar_ = params; }
    [[nodiscard]] const domain::LidarParams& lidarParams() const noexcept { return lidar_; }

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
    double cloudAccum_{0.0};   ///< 点云发布节拍累加器，与物理步解耦
    double lastWall_{0.0};
    bool snapshotDirty_{true};
    bool running_{false};
    std::uint32_t nextGhostId_{900000};   ///< 与场景参与者 id 分开，便于日志里一眼认出
};

}  // namespace adasim::app
