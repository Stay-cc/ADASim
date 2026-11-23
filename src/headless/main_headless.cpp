// 无界面闭环：不依赖 Qt，用来验证"感知—预测—规划—控制"这条链路本身是通的。
//
// 它有三个用途：
//   1. 在没装 Qt 的机器上证明核心逻辑可运行（CI 友好）；
//   2. 作为 6.1 实验的吞吐基线（每步耗时、重推演耗时）；
//   3. 回归测试：改动物理或时间轴后，跑一遍即可发现数值漂移。
//
// 控制器刻意写得很笨（Stanley + 比例调速），因为它不是研究对象，
// 只是用来产生"车确实在动、且动得符合预期"的证据。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

#include "application/ExperimentRunner.h"
#include "domain/model/ControlCommand.h"
#include "domain/port/ITransportPort.h"
#include "domain/sensor/LidarModel.h"
#include "domain/simulation/PhysicsEngine.h"
#include "domain/timeline/TimelineBuffer.h"

namespace {

/// 通信替身：把报文计数而不真正发送。
/// 这正好演示了端口倒置的用处——闭环可以在没有任何网络的情况下验证。
class CountingTransport final : public adasim::domain::ITransportPort {
public:
    void publishCloud(const adasim::domain::PointCloudPacket& packet) override {
        ++cloudPackets;
        lastCloudPoints = packet.pointCount();
    }
    void publishState(const adasim::domain::StatePacket&) override { ++statePackets; }
    bool pollCommand(adasim::domain::ControlCommand& out) override {
        // 无外部算法时直接回放内置控制器的输出，
        // 让"指令来自 Socket"这条路径在类型上被走到。
        if (!queued) {
            return false;
        }
        out = pending;
        queued = false;
        ++commandsConsumed;
        return true;
    }
    [[nodiscard]] bool linkAlive() const noexcept override { return true; }

    void push(const adasim::domain::ControlCommand& cmd) {
        pending = cmd;
        queued = true;
    }

    int cloudPackets{0};
    int statePackets{0};
    int commandsConsumed{0};
    std::size_t lastCloudPoints{0};
    adasim::domain::ControlCommand pending{};
    bool queued{false};
};

/// Stanley 横向控制 + 自适应巡航纵向控制。
///
/// 纵向刻意不用定速：定速会让自车在 30 s 内追尾前车，演示直接变成碰撞现场。
/// 这里用最简的间距误差法（期望间距 = 车头时距 × 车速 + 最小间距），
/// 它只有两行，但足以让自车稳定跟住前车——这也说明纵向控制不难，
/// 难的是横向与决策。
adasim::domain::ControlCommand control(const adasim::domain::WorldState& s,
                                       double cruiseSpeed) {
    adasim::domain::ControlCommand cmd;

    // 期望路径为 y = 0。横向误差取 -y，航向误差取 -theta（目标航向为 0）。
    const double latError = -s.ego.y;
    const double headingError = -s.ego.theta;

    // 低速时 atan(k*e/v) 会发散，分母加下限；这是 Stanley 的常规做法。
    const double softened = std::max(std::abs(s.egoSpeed), 1.0);
    cmd.steer = headingError + std::atan(1.2 * latError / softened);

    // 纵向：找同车道最近的前车，按间距误差调节目标车速。
    double targetSpeed = cruiseSpeed;
    const adasim::domain::EntityState* lead = nullptr;
    double bestGap = 1e9;
    for (const auto& e : s.obstacles) {
        const double gap = e.pose.x - s.ego.x - 0.5 * (e.length + 4.5);
        if (gap > 0.0 && gap < bestGap && std::abs(e.pose.y - s.ego.y) < 1.8) {
            bestGap = gap;
            lead = &e;
        }
    }
    if (lead != nullptr) {
        constexpr double kHeadway = 1.5;  // s
        constexpr double kStandstill = 5.0;
        const double desiredGap = kHeadway * s.egoSpeed + kStandstill;
        const double adjust = 0.5 * (bestGap - desiredGap);
        targetSpeed = std::clamp(lead->speed + adjust, 0.0, cruiseSpeed);
    }

    cmd.targetSpeed = targetSpeed;
    cmd.targetAccel = 1.5 * (targetSpeed - s.egoSpeed);

    cmd.simTime = s.simTime;
    cmd.echoFrameId = s.frameId;
    return cmd;
}

}  // namespace

int main() {
    using Clock = std::chrono::steady_clock;

    adasim::domain::PhysicsEngine physics;
    adasim::domain::TimelineBuffer buffer(1500, 25);
    CountingTransport transport;
    adasim::domain::LidarParams lidar;
    lidar.pointsPerMeter = 3.0;  // 约每辆车 40~60 点，够用来验证点云链路

    // 初始状态：自车 8 m/s，前方 25 m 一辆 4 m/s 的慢车。
    adasim::domain::WorldState state;
    state.egoSpeed = 8.0;

    adasim::domain::EntityState lead;
    lead.id = 1;
    lead.kind = adasim::domain::EntityKind::Car;
    lead.pose.x = 25.0;
    lead.speed = 4.0;
    state.obstacles.push_back(lead);

    constexpr double kDt = 0.02;
    constexpr int kSteps = 1500;  // 30 s
    const double cruiseSpeed = 8.0;

    buffer.append(state);

    double maxAbsAccel = 0.0;
    double minTtc = 1e9;
    std::size_t maxPoints = 0;
    adasim::domain::WorldSnapshot snapshot;

    const auto t0 = Clock::now();

    for (int i = 0; i < kSteps; ++i) {
        auto cmd = control(state, cruiseSpeed);
        transport.push(cmd);

        adasim::domain::ControlCommand applied;
        if (transport.pollCommand(applied)) {
            cmd = applied;
        }

        state = physics.step(state, kDt, cmd);
        buffer.append(state);

        maxAbsAccel = std::max(maxAbsAccel, std::abs(state.egoAccel));
        minTtc = std::min(minTtc, adasim::app::ExperimentRunner::timeToCollision(state));

        // 每 5 帧生成一次点云：模拟 10 Hz 的雷达输出，
        // 也顺便压一下轮廓采样的开销。
        if (i % 5 == 0) {
            adasim::domain::LidarModel::sample(state, lidar, snapshot);

            adasim::domain::PointCloudPacket cloud;
            cloud.frameId = state.frameId;
            cloud.simTime = state.simTime;
            cloud.px = snapshot.px;
            cloud.py = snapshot.py;
            cloud.intensity = snapshot.intensity;
            transport.publishCloud(cloud);

            maxPoints = std::max(maxPoints, snapshot.pointCount());

            adasim::domain::StatePacket sp;
            sp.frameId = state.frameId;
            sp.simTime = state.simTime;
            sp.ego = state.ego;
            sp.egoSpeed = state.egoSpeed;
            sp.obstacles = state.obstacles;
            transport.publishState(sp);
        }
    }

    const auto t1 = Clock::now();
    const double wallSeconds = std::chrono::duration<double>(t1 - t0).count();

    std::printf("=== 闭环运行结果 ===\n");
    std::printf("步数            : %d（%.1f s 仿真时间）\n", kSteps, kSteps * kDt);
    std::printf("真实耗时        : %.3f s\n", wallSeconds);
    std::printf("单步均耗时      : %.1f us\n", wallSeconds * 1e6 / kSteps);
    std::printf("实时倍率        : %.1f x\n", (kSteps * kDt) / wallSeconds);
    std::printf("最终位置        : x=%.2f m  y=%.3f m  theta=%.4f rad\n",
                state.ego.x, state.ego.y, state.ego.theta);
    std::printf("最终车速        : %.2f m/s\n", state.egoSpeed);
    std::printf("最大纵向加速度  : %.2f m/s^2\n", maxAbsAccel);
    if (minTtc > 1e8) {
        std::printf("最小 TTC        : 全程未接近目标\n");
    } else {
        std::printf("最小 TTC        : %.2f s\n", minTtc);
    }
    std::printf("点云峰值        : %zu 点\n", maxPoints);
    std::printf("报文            : 点云 %d 包 / 状态 %d 包 / 指令 %d 条\n",
                transport.cloudPackets, transport.statePackets, transport.commandsConsumed);
    std::printf("缓冲            : %zu 帧，估算 %.2f MB\n", buffer.size(),
                buffer.approximateBytes() / (1024.0 * 1024.0));

    // ---- 回溯一致性：跳到第 300 帧，重推演到第 320 帧，与直跑结果逐位比较 ----
    std::printf("\n=== 回溯一致性 ===\n");
    const auto direct = buffer.at(320);
    const auto plan = buffer.planSeek(320);
    if (!direct.has_value() || !plan.has_value()) {
        std::printf("[FAIL] 无法取得第 320 帧或其回溯方案\n");
        return 1;
    }

    adasim::domain::WorldState replay = plan->baseState;
    for (std::uint64_t i = 0; i < plan->replaySteps; ++i) {
        replay = physics.step(replay, kDt, control(replay, cruiseSpeed));
    }

    const double dx = std::abs(replay.ego.x - direct->state.ego.x);
    const double dy = std::abs(replay.ego.y - direct->state.ego.y);
    const double dth = std::abs(replay.ego.theta - direct->state.ego.theta);

    std::printf("起点快照        : 第 %llu 帧，重推演 %llu 步\n",
                static_cast<unsigned long long>(plan->baseFrame),
                static_cast<unsigned long long>(plan->replaySteps));
    std::printf("位置偏差        : dx=%.3e  dy=%.3e  dtheta=%.3e\n", dx, dy, dth);

    const bool consistent = dx < 1e-9 && dy < 1e-9 && dth < 1e-9;
    std::printf("[%s] 重推演与直跑逐位一致\n", consistent ? "PASS" : "FAIL");

    // 重推演耗时：这是 5.3.3 节"回溯延迟"的实测值来源。
    const auto t2 = Clock::now();
    adasim::domain::WorldState again = plan->baseState;
    for (int rep = 0; rep < 50; ++rep) {
        again = plan->baseState;
        for (std::uint64_t i = 0; i < plan->replaySteps; ++i) {
            again = physics.step(again, kDt, control(again, cruiseSpeed));
        }
    }
    const auto t3 = Clock::now();
    const double perSeekMs = std::chrono::duration<double, std::milli>(t3 - t2).count() / 50.0;
    std::printf("单次回溯耗时    : %.3f ms（重推演 %llu 步）\n", perSeekMs,
                static_cast<unsigned long long>(plan->replaySteps));

    return consistent ? 0 : 1;
}
