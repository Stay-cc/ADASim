// 断线兜底的手工验证程序（不进 CMake，随用随编）。
// 编译（MinGW，从项目根目录执行）：
//   g++ -std=c++17 -O2 -Isrc tests/standby_guard_check.cpp \
//       build/qt/src/application/libadasim_app.a build/qt/src/domain/libadasim_domain.a \
//       -o build/verify/standby_guard_check.exe
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "application/StandbyGuard.h"
#include "domain/model/ForwardGap.h"
#include "domain/simulation/PhysicsEngine.h"

using namespace adasim;

namespace {

struct LoopResult {
    double minGap{1e9};
    double finalGap{0.0};
    double finalSpeed{0.0};
    double peakDecel{0.0};    ///< 峰值减速度，正数
    double peakJerk{0.0};     ///< 单步 |Δa|/dt 的最大值
    int interventions{0};
    int brakeFrames{0};
};

/// 无客户端场景下的闭环：算法进程缺席 → 底座滑行 + 兜底。
LoopResult runLoop(double vEgo, double vLead, double gapCenter, double seconds,
                   bool guardOn, app::StandbyGuardParams p = {}) {
    domain::WorldState s;
    s.egoSpeed = vEgo;
    domain::EntityState lead;
    lead.id = 1;
    lead.pose.x = gapCenter;
    lead.speed = vLead;
    lead.length = 4.5;
    s.obstacles.push_back(lead);

    domain::PhysicsEngine physics;
    app::StandbyGuard guard(p);
    LoopResult r;

    const double dt = 0.02;
    const int steps = static_cast<int>(seconds / dt);
    double lastAccel = 0.0;

    for (int i = 0; i < steps; ++i) {
        domain::ControlCommand cmd;   // 断线保持：滑行
        cmd.targetAccel = 0.0;
        if (!guardOn) {
            // 对照组：同样滑行，只是不调用兜底
        } else {
            const bool braking = guard.apply(s, cmd, dt);
            if (braking) ++r.brakeFrames;
        }

        const auto gap = domain::nearestForwardTarget(s);
        r.minGap = std::min(r.minGap, gap.bumperGap);
        r.peakDecel = std::max(r.peakDecel, -cmd.targetAccel);
        r.peakJerk = std::max(r.peakJerk, std::abs(cmd.targetAccel - lastAccel) / dt);
        lastAccel = cmd.targetAccel;

        s = physics.step(s, dt, cmd);
    }

    const auto end = domain::nearestForwardTarget(s);
    r.finalGap = end.bumperGap;
    r.finalSpeed = s.egoSpeed;
    r.minGap = std::min(r.minGap, end.bumperGap);
    r.interventions = static_cast<int>(guard.interventions());
    return r;
}

}  // namespace

int main() {
    int fails = 0;
    auto expect = [&](const char* name, bool ok, const char* detail) {
        std::printf("[%s] %-20s %s\n", ok ? "PASS" : "FAIL", name, detail);
        if (!ok) ++fails;
    };

    // 1. 前方无目标：不该凭空刹车
    {
        domain::WorldState s;
        s.egoSpeed = 8.0;
        app::StandbyGuard g;
        domain::ControlCommand c;
        const bool braking = g.apply(s, c, 0.02);
        expect("空场景不介入", !braking && c.targetAccel == 0.0, "");
    }

    // 2. 前车很远：不该介入
    {
        domain::WorldState s;
        s.egoSpeed = 8.0;
        domain::EntityState lead;
        lead.pose.x = 60.0;
        lead.speed = 4.0;
        lead.length = 4.5;
        s.obstacles.push_back(lead);
        app::StandbyGuard g;
        domain::ControlCommand c;
        bool braking = false;
        for (int i = 0; i < 100; ++i) {
            c = domain::ControlCommand{};
            braking = g.apply(s, c, 0.02) || braking;
        }
        const auto gap = domain::nearestForwardTarget(s);
        char buf[96];
        std::snprintf(buf, sizeof buf, "净距 %.1f m，2 s 后 a = %.2f", gap.bumperGap,
                      c.targetAccel);
        expect("远处前车不介入", !braking, buf);
    }

    // 3. 主场景闭环：滑行 20 s，靠兜底停住
    {
        const auto r = runLoop(8.0, 4.0, 25.0, 20.0, true);
        char buf[176];
        std::snprintf(buf, sizeof buf,
                      "最小净距 %.2f m，末态 %.2f m，车速 %.2f m/s，峰值 %.2f m/s^2，"
                      "加加速度 %.1f m/s^3，介入 %d 次",
                      r.minGap, r.finalGap, r.finalSpeed, r.peakDecel, r.peakJerk,
                      r.interventions);
        expect("20 s 不追尾", r.minGap > 2.0, buf);
        expect("稳态车距合理", r.finalGap > 5.0 && r.finalGap < 14.0, buf);
        expect("介入不反复", r.interventions <= 2, buf);
        expect("减速度不越限", r.peakDecel <= 6.0 + 1e-9, buf);
        expect("指令连续无跳变", r.peakJerk <= 8.0 * 1.05, buf);
    }

    // 4. 高速接近静止障碍：硬制动层要被调用
    {
        const auto r = runLoop(20.0, 0.0, 60.0, 20.0, true);
        char buf[160];
        std::snprintf(buf, sizeof buf, "最小净距 %.2f m，末态 %.2f m，峰值 %.2f m/s^2，介入 %d 次",
                      r.minGap, r.finalGap, r.peakDecel, r.interventions);
        expect("高速接近能停下", r.minGap > 0.0, buf);
    }

    // 5. 已经贴近且前车静止：净距 11.5 m，8 m/s 正好在 6 m/s^2 的能力边界内
    {
        const auto r = runLoop(8.0, 0.0, 16.0, 20.0, true);
        char buf[160];
        std::snprintf(buf, sizeof buf, "最小净距 %.2f m，峰值 %.2f m/s^2", r.minGap,
                      r.peakDecel);
        expect("近距离能停住", r.minGap > 0.0, buf);
    }

    // 6. 对照：同一场景去掉兜底，确认确实会撞
    {
        const auto r = runLoop(8.0, 4.0, 25.0, 20.0, false);
        char buf[96];
        std::snprintf(buf, sizeof buf, "纯滑行最小净距 %.2f m", r.minGap);
        expect("对照组确实追尾", r.minGap <= 0.0, buf);
    }

    std::printf(fails == 0 ? "\n全部通过\n" : "\n%d 项失败\n", fails);
    return fails == 0 ? 0 : 1;
}
