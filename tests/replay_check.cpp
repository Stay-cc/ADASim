// 回溯一致性检查（E3 实验的取数程序）。
//
// 口径：
//   1. 先在一条含"跟驰 + 切入 + 匀速"三车的轨迹上直跑到 1200 帧；
//   2. 在窗口内均匀取 19 个目标帧，逐个走 planSeek → 从最近快照重推演；
//   3. 把重推演结果与直跑时留在环缓冲里的同一帧逐位比较。
//
// 为什么"逐位相等"才是合格线：状态推进是纯确定性的（同一初值 + 同一指令
// 序列），只要重推演路径与直跑路径等价，浮点结果就必须一模一样。若允许
// 1e-3 的容差，任何一处"少走一步、多积一次"的实现错误都会被容差盖住，
// 而这类错误会让回放画面在几十秒后慢慢漂移——正是回溯最该避免的失效。
//
// 编译（从项目根目录执行，MinGW）：
//   g++ -std=c++17 -O2 -Isrc tests/replay_check.cpp \
//       build/qt/src/application/libadasim_app.a build/qt/src/domain/libadasim_domain.a \
//       -o build/verify/replay_check.exe

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <vector>

#include "domain/model/ControlCommand.h"
#include "domain/model/ForwardGap.h"
#include "domain/simulation/ActorBehavior.h"
#include "domain/simulation/PhysicsEngine.h"
#include "domain/timeline/TimelineBuffer.h"

using namespace adasim::domain;

namespace {

constexpr double kDt = 0.02;
constexpr int kSteps = 1200;          // 24 s
constexpr std::uint64_t kSnapshotEvery = 25;

int fails = 0;

void expect(const char* name, bool ok, const char* detail) {
    std::printf("  [%s] %s %s\n", ok ? "通过" : "失败", name, detail);
    if (!ok) {
        ++fails;
    }
}

/// 与实验场景 replay_mixed.json 同构的初始状态：三车、三种行为。
WorldState makeInitial() {
    WorldState s;
    s.egoSpeed = 10.0;
    s.friction = 0.85;

    EntityState lead;
    lead.id = 1;
    lead.pose = Pose{42.0, 0.0, 0.0};
    lead.speed = 8.0;
    lead.behavior.kind = BehaviorKind::CarFollowing;
    lead.behavior.desiredSpeed = 8.0;

    EntityState cut;
    cut.id = 2;
    cut.pose = Pose{6.0, -3.5, 0.0};
    cut.speed = 12.0;
    cut.behavior.kind = BehaviorKind::CutIn;
    cut.behavior.originY = -3.5;
    cut.behavior.targetY = 0.0;
    cut.behavior.triggerSeconds = 5.0;
    cut.behavior.laneChangeSeconds = 3.2;
    cut.behavior.brakeAccel = 1.5;
    cut.behavior.speedFloor = 5.0;

    EntityState other;
    other.id = 3;
    other.pose = Pose{25.0, -3.5, 0.0};
    other.speed = 9.0;
    other.behavior.kind = BehaviorKind::Constant;

    s.obstacles = {lead, cut, other};
    return s;
}

/// 直跑时使用的控制律。重推演必须复用同一函数，否则比的是两套控制器。
ControlCommand control(const WorldState& s) {
    ControlCommand cmd;
    const ForwardGap gap = nearestForwardTarget(s, 2.5);
    cmd.steer = 0.0;
    if (gap.valid) {
        const double desired = 2.0 + 1.4 * s.egoSpeed;
        cmd.targetAccel = 0.4 * (gap.bumperGap - desired) - 0.6 * gap.closingSpeed;
    } else {
        cmd.targetAccel = 0.5 * (10.0 - s.egoSpeed);
    }
    cmd.targetAccel = std::clamp(cmd.targetAccel, -6.0, 3.0);
    cmd.echoFrameId = s.frameId;
    cmd.simTime = s.simTime;
    return cmd;
}

double poseError(const WorldState& a, const WorldState& b) {
    return std::max({std::abs(a.ego.x - b.ego.x), std::abs(a.ego.y - b.ego.y),
                     std::abs(a.ego.theta - b.ego.theta),
                     std::abs(a.egoSpeed - b.egoSpeed)});
}

}  // namespace

int main() {
    PhysicsEngine physics;
    TimelineBuffer buffer(1500, kSnapshotEvery);

    WorldState state = makeInitial();
    buffer.append(state);

    WorldState last = state;
    for (int i = 0; i < kSteps; ++i) {
        state = physics.step(state, kDt, control(state));
        buffer.append(state);
        last = state;
    }

    std::printf("=== 回溯一致性 ===\n");
    std::printf("直跑帧数        : %d（%.1f s）\n", kSteps + 1, kSteps * kDt);
    std::printf("快照间隔        : 每 %llu 帧\n",
                static_cast<unsigned long long>(kSnapshotEvery));
    std::printf("缓冲            : %zu 帧，估算 %.2f MB\n", buffer.size(),
                buffer.approximateBytes() / (1024.0 * 1024.0));

    // ---- 1. 均匀采样目标帧，逐帧比较 ----
    constexpr int kSamples = 19;
    double worstPose = 0.0;
    double worstActor = 0.0;
    std::uint64_t maxReplaySteps = 0;
    std::uint64_t totalReplaySteps = 0;
    int seekFailures = 0;
    int frameMismatch = 0;

    for (int k = 0; k < kSamples; ++k) {
        const FrameId target = 100 + static_cast<FrameId>(k) * 60;
        const auto direct = buffer.at(target);
        const auto plan = buffer.planSeek(target);
        if (!direct.has_value() || !plan.has_value()) {
            ++seekFailures;
            continue;
        }
        if (plan->targetFrame != target) {
            ++frameMismatch;   // 回溯定位必须落在被请求的那一帧
        }

        WorldState replay = plan->baseState;
        for (std::uint64_t i = 0; i < plan->replaySteps; ++i) {
            replay = physics.step(replay, kDt, control(replay));
        }

        worstPose = std::max(worstPose, poseError(replay, direct->state));

        // 参与者状态也要比：只比自车位姿会漏掉"行为推进没有回退"这类错误，
        // 表现为回放时旁车停在切入中途。
        if (replay.obstacles.size() != direct->state.obstacles.size()) {
            worstActor = 1e9;
        } else {
            for (std::size_t i = 0; i < replay.obstacles.size(); ++i) {
                const auto& a = replay.obstacles[i];
                const auto& b = direct->state.obstacles[i];
                worstActor = std::max({worstActor, std::abs(a.pose.x - b.pose.x),
                                       std::abs(a.pose.y - b.pose.y),
                                       std::abs(a.speed - b.speed)});
            }
        }

        maxReplaySteps = std::max(maxReplaySteps, plan->replaySteps);
        totalReplaySteps += plan->replaySteps;
    }

    std::printf("\n采样帧数        : %d\n", kSamples);
    std::printf("自车位姿最大偏差: %.3e\n", worstPose);
    std::printf("参与者最大偏差  : %.3e\n", worstActor);
    std::printf("重推演步数      : 最多 %llu，平均 %.1f\n",
                static_cast<unsigned long long>(maxReplaySteps),
                static_cast<double>(totalReplaySteps) / kSamples);

    {
        char buf[128];
        std::snprintf(buf, sizeof buf, "最大位姿偏差 %.2e", worstPose);
        expect("重推演与直跑逐位一致", worstPose == 0.0, buf);
    }
    {
        char buf[128];
        std::snprintf(buf, sizeof buf, "最大参与者偏差 %.2e", worstActor);
        expect("参与者状态一致", worstActor == 0.0, buf);
    }
    {
        char buf[96];
        std::snprintf(buf, sizeof buf, "失败 %d 次", seekFailures);
        expect("全部目标帧可定位", seekFailures == 0, buf);
    }
    {
        char buf[96];
        std::snprintf(buf, sizeof buf, "错帧 %d 次", frameMismatch);
        expect("定位误差不超过 1 个物理步长", frameMismatch == 0, buf);
    }

    // ---- 2. 回溯耗时 ----
    // 取窗口末尾（重推演步数最多、最费时的情况）重复 200 次取平均，
    // 单次测量受调度抖动影响太大，直接报单次值没有意义。
    // 取一个不落在快照点上的帧：正好命中快照时重推演步数为 0，
    // 测出来的是"读快照"而不是"重推演"，会给出 0 ms 的假结论。
    const auto heavy = buffer.planSeek(static_cast<FrameId>(kSteps - 1));
    if (heavy.has_value()) {
        constexpr int kReps = 200;
        const auto t0 = std::chrono::steady_clock::now();
        double sink = 0.0;
        for (int r = 0; r < kReps; ++r) {
            WorldState s = heavy->baseState;
            for (std::uint64_t i = 0; i < heavy->replaySteps; ++i) {
                s = physics.step(s, kDt, control(s));
            }
            sink += s.ego.x;
        }
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / kReps;
        std::printf("最坏回溯耗时    : %.3f ms（重推演 %llu 步，%.0f 步/ms）\n", ms,
                    static_cast<unsigned long long>(heavy->replaySteps),
                    heavy->replaySteps / std::max(1e-6, ms));
        std::printf("（校验和 %.3f，防止编译器把循环优化掉）\n", sink);
    }

    // ---- 3. 跨窗口回溯：跳到窗口开头，确认仍可正确定位 ----
    {
        const auto begin = buffer.planSeek(buffer.earliest());
        char buf[96];
        std::snprintf(buf, sizeof buf, "begin=%llu replay=%llu",
                      static_cast<unsigned long long>(buffer.earliest()),
                      begin.has_value()
                          ? static_cast<unsigned long long>(begin->replaySteps) : 0ULL);
        expect("窗口起点可回溯", begin.has_value() && begin->targetFrame == buffer.earliest(), buf);
    }

    std::printf(fails == 0 ? "\n全部通过\n" : "\n%d 项失败\n", fails);
    return fails == 0 ? 0 : 1;
}
