#include "application/SimController.h"

#include <algorithm>
#include <utility>

namespace adasim::app {

namespace {
constexpr int kMaxCatchUpSteps = 8;
}  // namespace

SimController::SimController(domain::PhysicsEngine& physics,
                             domain::TimelineBuffer& buffer,
                             domain::ITransportPort& transport,
                             domain::IClock& clock)
    : physics_(physics), buffer_(buffer), transport_(transport), clock_(clock) {
    lastWall_ = clock_.monotonicSeconds();
}

void SimController::reset(const domain::WorldState& initial) {
    current_ = initial;
    lastCommand_ = domain::ControlCommand{};
    standbyGuard_.reset();
    accumulator_ = 0.0;
    cloudAccum_ = 0.0;
    lastWall_ = clock_.monotonicSeconds();
    running_ = false;
    snapshot_.reset();
    buffer_.clear();
    buffer_.append(current_);
    buildSnapshotCache();
}

void SimController::start() {
    running_ = true;
    lastWall_ = clock_.monotonicSeconds();
}

void SimController::pause() { running_ = false; }

void SimController::toggle() {
    if (running_) {
        pause();
    } else {
        start();
    }
}

void SimController::setSpeedFactor(double factor) {
    speedFactor_ = std::clamp(factor, 0.1, 8.0);
}

void SimController::setFixedStep(double dt) {
    if (dt > 0.0) {
        fixedStep_ = dt;
    }
}

bool SimController::stepIfDue(double nowMonotonicSeconds) {
    if (!running_) {
        lastWall_ = nowMonotonicSeconds;
        return false;
    }
    const double elapsed = std::max(0.0, nowMonotonicSeconds - lastWall_);
    lastWall_ = nowMonotonicSeconds;
    accumulator_ += elapsed * speedFactor_;

    int steps = 0;
    while (accumulator_ >= fixedStep_ && steps < kMaxCatchUpSteps) {
        stepOnce();
        accumulator_ -= fixedStep_;
        ++steps;
    }
    if (steps == kMaxCatchUpSteps) {
        accumulator_ = 0.0;  // 追不上就丢弃，宁可跳帧也不要越拖越久
    }
    return steps > 0;
}

void SimController::stepOnce() {
    if (transport_.linkAlive()) {
        domain::ControlCommand cmd;
        if (transport_.pollCommand(cmd)) {
            lastCommand_ = cmd;
        }
        // 客户端回来了，兜底立即让出纵向控制权（介入次数保留，供统计用）
        standbyGuard_.release();
    } else {
        // 断线保持模式：不采纳旧指令，按上一拍状态滑行，避免状态错乱。
        lastCommand_.targetAccel = 0.0;

        // 但"滑行"本身并不安全：算法进程没接入时，自车会以初速一路匀速逼近
        // 前车（实测 6 s 内就贴到一起）。于是这里再加一层兜底制动——
        // 只在前车真的要撞上时介入，且只在无客户端时接管纵向控制。
        // 有客户端在线时绝不插手：否则感知失效、决策振荡这类该暴露的问题
        // 会被底座替算法刹住，闭环对比实验就白做了。
        standbyGuard_.apply(current_, lastCommand_, fixedStep_);
    }

    current_ = physics_.step(current_, fixedStep_, lastCommand_);
    buffer_.append(current_);
    snapshotDirty_ = true;
    publish();
}

bool SimController::seek(domain::FrameId target) {
    const auto plan = buffer_.planSeek(target);
    if (!plan.has_value()) {
        return false;
    }

    domain::WorldState state = plan->baseState;
    for (domain::FrameId f = plan->baseFrame + 1; f <= plan->targetFrame; ++f) {
        const auto rec = buffer_.at(f);
        if (!rec.has_value()) {
            break;
        }
        domain::ControlCommand cmd;
        cmd.echoFrameId = f - 1;
        cmd.steer = rec->state.steer;
        cmd.targetAccel = rec->state.egoAccel;
        state = physics_.step(state, fixedStep_, cmd);
    }

    state.frameId = plan->targetFrame;
    state.simTime = plan->baseState.simTime +
                    static_cast<double>(plan->replaySteps) * fixedStep_;

    current_ = state;
    accumulator_ = 0.0;
    lastWall_ = clock_.monotonicSeconds();

    // 回溯即回退写指针：继续仿真会覆盖该帧之后的历史
    buffer_.rewindTo(plan->targetFrame);
    snapshotDirty_ = true;
    return true;
}

void SimController::buildSnapshotCache() {
    auto snap = std::make_shared<domain::WorldSnapshot>();
    domain::LidarModel::sample(current_, lidar_, *snap);
    snapshot_ = std::move(snap);
    snapshotDirty_ = false;
}

std::shared_ptr<const domain::WorldSnapshot> SimController::snapshot() {
    // 懒重建：只有状态推进过才重新采样。渲染与点云发布各有自己的节拍，
    // 两者都来取快照，若每步都无条件重采，十万点下等于每秒多做 40 次
    // 无用功（0.47 ms × 50），纯属白烧 CPU。
    if (!snapshot_ || snapshotDirty_) {
        buildSnapshotCache();
    }
    return snapshot_;
}

void SimController::publish() {
    domain::StatePacket sp;
    sp.frameId = current_.frameId;
    sp.simTime = current_.simTime;
    sp.ego = current_.ego;
    sp.egoSpeed = current_.egoSpeed;
    sp.obstacles = current_.obstacles;
    transport_.publishState(sp);

    // 点云按传感器频率发布，不跟着物理步走。
    // 原因见 LidarParams::rateHz：十万点下每步发一次等于每秒 4.3 万个分片。
    // 用累加器而不是"每 N 步一次"，是因为 50 Hz / 10 Hz = 5 只是恰好整除；
    // 若物理步长或雷达频率改成不整除的组合，取模会固定丢掉半拍。
    const double period = lidar_.rateHz > 0.0 ? 1.0 / lidar_.rateHz : 0.0;
    cloudAccum_ += fixedStep_;
    if (period > 0.0 && cloudAccum_ + 1e-9 >= period) {
        cloudAccum_ -= period;
        if (cloudAccum_ < 0.0) {
            cloudAccum_ = 0.0;
        }

        const auto snap = snapshot();  // 脏则重建，不脏直接复用
        domain::PointCloudPacket cp;
        cp.frameId = current_.frameId;
        cp.simTime = current_.simTime;
        cp.px = snap->px;
        cp.py = snap->py;
        cp.intensity = snap->intensity;
        transport_.publishCloud(cp);
    }
}

}  // namespace adasim::app
