// 切入行为的手工用例（不进 CMake，随用随编）。
//
// 编译（在项目根目录，需先构建 build/qt）：
//   g++ -std=c++17 -O2 -Isrc tests/cut_in_check.cpp \
//       build/qt/src/application/libadasim_app.a \
//       build/qt/src/domain/libadasim_domain.a -o build/verify/cut_in_check.exe
//
// 它回答三个问题：
//   1. 切入的几何对不对（起点不横移、终点贴住目标车道、航向两端回零）；
//   2. 这个场景到底危不危险（同一初值，自车不减速会不会撞）；
//   3. 行为推进是否与步长/重推演无关（从快照重跑必须逐位一致）。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "application/ScenarioService.h"
#include "domain/model/ForwardGap.h"
#include "domain/port/IPersistence.h"
#include "domain/simulation/ActorBehavior.h"
#include "domain/simulation/PhysicsEngine.h"
#include "domain/timeline/TimelineBuffer.h"

namespace {

int g_failed = 0;

void check(bool ok, const char* what, const std::string& detail = {}) {
    std::printf("  [%s] %s%s%s\n", ok ? "通过" : "失败", what, detail.empty() ? "" : "  ",
                detail.c_str());
    if (!ok) {
        ++g_failed;
    }
}

/// 场景仓储替身：把模板直接交给 ScenarioService，绕开 Qt 与文件系统。
class MemoryStore final : public adasim::domain::IPersistence {
public:
    explicit MemoryStore(adasim::domain::ScenarioTemplate tmpl) : tmpl_(std::move(tmpl)) {}

    std::optional<adasim::domain::ScenarioTemplate> load(const std::string&) override {
        return tmpl_;
    }
    bool save(const std::string&, const adasim::domain::ScenarioTemplate&) override {
        return true;
    }
    std::vector<std::string> listTemplates() const override { return {tmpl_.id}; }

private:
    adasim::domain::ScenarioTemplate tmpl_;
};

adasim::domain::ScenarioTemplate makeCutInTemplate() {
    using namespace adasim::domain;
    ScenarioTemplate tmpl;
    tmpl.id = "cut_in_brake";
    tmpl.laneCount = 3;
    tmpl.laneWidth = 3.5;
    tmpl.friction = 0.8;
    tmpl.durationSeconds = 25.0;

    ActorSpec lead;
    lead.pose = Pose{45.0, 0.0, 0.0};
    lead.speed = 6.0;
    lead.behavior = BehaviorKind::CarFollowing;
    tmpl.actors.push_back(lead);

    ActorSpec cutter;
    cutter.pose = Pose{12.0, 3.5, 0.0};
    cutter.speed = 9.0;
    cutter.behavior = BehaviorKind::CutIn;
    cutter.triggerSeconds = 2.5;
    cutter.laneChangeSeconds = 3.5;
    cutter.brakeAccel = 2.5;
    cutter.speedFloor = 1.5;
    tmpl.actors.push_back(cutter);

    return tmpl;
}

/// 纵向控制直接用领域层的 IDM（与 Python 算法侧同一套公式与参数）。
/// 早先这里用"间距误差比例调速"，它没有相对速度阻尼，会在净距 0.5 m 处
/// 找到平衡点、以 0.5 m/s 的速度慢慢蹭上前车——那是控制律的缺陷，
/// 与场景无关，不该拿来判断切入是否合理。
adasim::domain::ControlCommand followControl(const adasim::domain::WorldState& s,
                                             double cruise) {
    using namespace adasim::domain;
    ControlCommand cmd;
    cmd.steer = -s.ego.y / std::max(std::abs(s.egoSpeed), 1.0) * 0.5;

    EntityState self;
    self.id = 0;                       // 自车不在 obstacles 里，用 0 号不会撞号
    self.pose = s.ego;
    self.speed = s.egoSpeed;
    self.length = kEgoBodyLength;
    self.behavior.desiredSpeed = cruise;

    cmd.targetSpeed = cruise;
    cmd.targetAccel = idmAccelFor(s, self);
    return cmd;
}

struct RunResult {
    double minBumperGap{1e9};
    double minTtc{1e9};
    bool collided{false};
    double actorFinalSpeed{0.0};
    double actorMaxLateralDeviation{0.0};
    double actorYatTrigger{0.0};
    double actorYatEnd{0.0};
    double actorMaxTheta{0.0};
    double actorThetaAtEnd{0.0};
};

RunResult runClosedLoop(const adasim::domain::WorldState& initial, double seconds, bool slowDown) {
    using namespace adasim::domain;
    PhysicsEngine physics;
    WorldState s = initial;
    RunResult r;

    const int steps = static_cast<int>(std::lround(seconds / 0.02));
    for (int i = 0; i < steps; ++i) {
        ControlCommand cmd;
        if (slowDown) {
            cmd = followControl(s, 8.0);
        } else {
            cmd.targetAccel = 0.0;   // 对照组：完全不减速
        }
        s = physics.step(s, 0.02, cmd);

        const ForwardGap gap = nearestForwardTarget(s);
        if (gap.valid) {
            r.minBumperGap = std::min(r.minBumperGap, gap.bumperGap);
        }
        r.minTtc = std::min(r.minTtc, criticalForwardTarget(s).ttc);

        if (!s.obstacles.empty()) {
            const EntityState& a = s.obstacles.back();
            r.actorFinalSpeed = a.speed;
            r.actorMaxTheta = std::max(r.actorMaxTheta, std::abs(a.pose.theta));
            if (std::abs(s.simTime - 2.48) < 1e-9) {
                r.actorYatTrigger = a.pose.y;
            }
            if (std::abs(s.simTime - 6.0) < 1e-9) {
                r.actorYatEnd = a.pose.y;
            }
            if (std::abs(s.simTime - 25.0) < 1e-9) {
                r.actorThetaAtEnd = a.pose.theta;
            }
        }
    }
    r.collided = r.minBumperGap <= 0.0;
    return r;
}

}  // namespace

int main() {
    using namespace adasim::domain;

    std::printf("=== 切入行为用例 ===\n");
    const ScenarioTemplate tmpl = makeCutInTemplate();
    MemoryStore store(tmpl);
    adasim::app::ScenarioService service(store);

    const auto issues = service.validate(tmpl);
    check(issues.empty(), "场景通过校验", issues.empty() ? "" : issues.front());

    const WorldState initial = service.buildInitialState(tmpl);
    const EntityState& cutter = initial.obstacles.at(1);
    check(cutter.behavior.kind == BehaviorKind::CutIn, "行为随状态实例化");
    check(std::abs(cutter.behavior.targetY - 0.0) < 1e-9, "目标车道由车道宽推出",
          "targetY=" + std::to_string(cutter.behavior.targetY));

    // ---- 1. 横向位移曲线 ----
    std::printf("\n-- 横向曲线 --\n");
    check(std::abs(cutInLateralRatio(0.0)) < 1e-12, "起点位移为零");
    check(std::abs(cutInLateralRatio(1.0) - 1.0) < 1e-12, "终点位移为一");
    check(std::abs(cutInLateralRatio(0.5) - 0.5) < 1e-12, "中点位移过半");

    // 端点斜率（横向速度）应为零，否则航向会跳变
    const double d0 = (cutInLateralRatio(1e-4) - cutInLateralRatio(0.0)) / 1e-4;
    const double d1 = (cutInLateralRatio(1.0) - cutInLateralRatio(1.0 - 1e-4)) / 1e-4;
    check(std::abs(d0) < 1e-2 && std::abs(d1) < 1e-2, "端点横向速度为零（航向连续）",
          "d0=" + std::to_string(d0) + " d1=" + std::to_string(d1));

    // ---- 2. 触发前的静止性 ----
    std::printf("\n-- 时序 --\n");
    {
        PhysicsEngine physics;
        WorldState s = initial;
        ControlCommand none;
        for (int i = 0; i < 120; ++i) {          // 2.4 s，未到触发时刻
            s = physics.step(s, 0.02, none);
        }
        const double y = s.obstacles.back().pose.y;
        check(std::abs(y - 3.5) < 1e-9, "触发前不横移", "y=" + std::to_string(y));
        check(std::abs(s.obstacles.back().pose.theta) < 1e-9, "触发前航向为零");
    }

    // ---- 3. 闭环：有减速 vs 无减速 ----
    std::printf("\n-- 闭环 --\n");
    const RunResult guarded = runClosedLoop(initial, 25.0, true);
    const RunResult naive = runClosedLoop(initial, 25.0, false);

    check(std::abs(guarded.actorYatEnd) < 1e-6, "平移结束时贴住目标车道",
          "y=" + std::to_string(guarded.actorYatEnd));
    check(guarded.actorMaxTheta > 0.05, "平移过程中航向非零（车是斜着切进来）",
          "maxθ=" + std::to_string(guarded.actorMaxTheta));
    check(std::abs(guarded.actorThetaAtEnd) < 1e-9, "平移结束后航向回零",
          "θ=" + std::to_string(guarded.actorThetaAtEnd));
    check(guarded.actorFinalSpeed < 2.0, "切入后减速到接近目标车速",
          "v=" + std::to_string(guarded.actorFinalSpeed));
    check(guarded.minBumperGap > 2.0, "自车减速后未追尾",
          "最小净距=" + std::to_string(guarded.minBumperGap) + " m");
    check(naive.collided, "对照组（不减速）发生碰撞，说明场景确实危险",
          "最小净距=" + std::to_string(naive.minBumperGap) + " m");

    // ---- 4. 重推演一致性 ----
    std::printf("\n-- 回溯一致性 --\n");
    {
        PhysicsEngine physics;
        TimelineBuffer buffer(1500, 25);
        WorldState s = initial;
        buffer.append(s);
        for (int i = 0; i < 800; ++i) {          // 16 s，跨越触发时刻
            s = physics.step(s, 0.02, followControl(s, 8.0));
            buffer.append(s);
        }

        const auto direct = buffer.at(700);
        const auto plan = buffer.planSeek(700);
        bool same = direct.has_value() && plan.has_value();
        if (same) {
            WorldState replay = plan->baseState;
            for (std::uint64_t i = 0; i < plan->replaySteps; ++i) {
                replay = physics.step(replay, 0.02, followControl(replay, 8.0));
            }
            const EntityState& a = replay.obstacles.back();
            const EntityState& b = direct->state.obstacles.back();
            same = std::abs(a.pose.x - b.pose.x) < 1e-12 &&
                   std::abs(a.pose.y - b.pose.y) < 1e-12 &&
                   std::abs(a.pose.theta - b.pose.theta) < 1e-12 &&
                   std::abs(a.speed - b.speed) < 1e-12;
        }
        check(same, "切入选段重推演与直跑一致");
    }

    std::printf("\n%s（失败 %d 项）\n", g_failed == 0 ? "全部通过" : "存在失败", g_failed);
    return g_failed == 0 ? 0 : 1;
}
