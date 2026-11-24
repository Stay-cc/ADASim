// 沙盒注入（雷达欺骗）检查。
//
// 要守的不变量有四条，缺一条这个功能就不成立：
//   1. 注入了目标，点云里必须真的出现它的轮廓——否则算法看不见，"所点即所得"是假的；
//   2. StatePacket.obstacles 里**不能**出现它——进了真值列表就等于给答案，
//      测不出感知链路有没有在工作；
//   3. 回看到注入之前，这个目标必须消失——注入是"某一时刻发生的事"，不是场景属性；
//   4. 回看不销毁历史（previewSeek），只有从这里继续跑（seek/start）才截断。
//
// 第 4 条与第 1 条同样重要：行车记录仪式拖拽的前提就是来回拖不丢数据。

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <vector>

#include "application/SimController.h"
#include "domain/model/WorldState.h"
#include "domain/port/IClock.h"
#include "domain/port/ITransportPort.h"
#include "domain/simulation/PhysicsEngine.h"
#include "domain/timeline/TimelineBuffer.h"

namespace {

int g_failed = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        std::printf("  [失败] %s\n", what);
        ++g_failed;
    } else {
        std::printf("  [通过] %s\n", what);
    }
}

class FakeClock final : public adasim::domain::IClock {
public:
    double monotonicSeconds() const noexcept override { return t_; }
    std::uint64_t monotonicNanos() const noexcept override {
        return static_cast<std::uint64_t>(t_ * 1e9);
    }
    void advance(double dt) { t_ += dt; }

private:
    double t_{0.0};
};

/// 只记录、不回发指令的传输替身。linkAlive 为 false，
/// 于是 SimController 会走 standbyGuard 分支——但那不影响注入与点云。
class RecordingTransport final : public adasim::domain::ITransportPort {
public:
    void publishCloud(const adasim::domain::PointCloudPacket& p) override { lastCloud = p; }
    void publishState(const adasim::domain::StatePacket& p) override { lastState = p; }
    [[nodiscard]] bool pollCommand(adasim::domain::ControlCommand&) override { return false; }
    [[nodiscard]] bool linkAlive() const noexcept override { return false; }

    adasim::domain::PointCloudPacket lastCloud{};
    adasim::domain::StatePacket lastState{};
};

adasim::domain::WorldState makeWorld() {
    adasim::domain::WorldState s;
    s.frameId = 0;
    s.simTime = 0.0;
    s.ego = adasim::domain::Pose{0.0, 0.0, 0.0};
    s.egoSpeed = 10.0;
    s.laneCount = 3;
    s.laneWidth = 3.5;
    return s;
}

/// 点云里落在给定矩形内的点数。
std::size_t pointsInBox(const adasim::domain::PointCloudPacket& cloud,
                        double x0, double x1, double y0, double y1) {
    std::size_t n = 0;
    for (std::size_t i = 0; i < cloud.pointCount(); ++i) {
        const double x = cloud.px[i];
        const double y = cloud.py[i];
        if (x >= x0 && x <= x1 && y >= y0 && y <= y1) {
            ++n;
        }
    }
    return n;
}

bool hasObstacleId(const adasim::domain::StatePacket& pkt, std::uint32_t id) {
    for (const auto& e : pkt.obstacles) {
        if (e.id == id) {
            return true;
        }
    }
    return false;
}

}  // namespace

int main() {
    std::printf("== 沙盒注入与雷达欺骗 ==\n");

    adasim::domain::PhysicsEngine physics;
    adasim::domain::TimelineBuffer buffer(600, 25);
    RecordingTransport transport;
    FakeClock clock;
    adasim::app::SimController sim(physics, buffer, transport, clock);

    adasim::domain::LidarParams lidar;
    lidar.range = 60.0;
    lidar.pointsPerMeter = 2.0;
    lidar.rateHz = 10.0;
    sim.setLidarParams(lidar);

    sim.reset(makeWorld());
    sim.start();

    // 注入前：空场景的点云必须是 0 点，否则后面"多出来的点"归因不明。
    for (int i = 0; i < 10; ++i) {
        clock.advance(0.02);
        sim.stepIfDue(clock.monotonicSeconds());
    }
    check(transport.lastCloud.pointCount() == 0, "空沙盒的点云为 0 点");
    check(transport.lastState.obstacles.empty(), "空沙盒的真值列表为空");

    // 在自车正前方 30 m 注入一辆车（4.5 × 1.9）。
    const std::uint32_t id = sim.injectGhost(30.0, 0.0, 0.0, adasim::domain::EntityKind::Car);
    check(sim.ghostCount() == 1, "注入后 ghost 计数为 1");

    for (int i = 0; i < 10; ++i) {
        clock.advance(0.02);
        sim.stepIfDue(clock.monotonicSeconds());
    }

    const auto cloud = transport.lastCloud;
    check(cloud.pointCount() > 0, "注入后点云非空");
    const std::size_t inBox = pointsInBox(cloud, 27.5, 32.5, -1.2, 1.2);
    check(inBox >= 10, "点云里出现注入目标的轮廓（>=10 点落在包围盒内）");
    // 强度衰减与真实障碍物同一条公式，不给算法留"一眼看穿"的破绽
    float minI = 1.0f;
    for (std::size_t i = 0; i < cloud.pointCount(); ++i) {
        minI = std::min(minI, cloud.intensity[i]);
    }
    check(minI < 1.0f, "注入目标的点带距离衰减强度");

    // 核心不变量：真值列表里没有它。
    check(!hasObstacleId(transport.lastState, id), "StatePacket 真值列表不含注入目标");
    check(transport.lastState.obstacles.empty(), "注入不会污染真实参与者列表");

    // 再跑一段：ghost 不参与物理推进，位置不变，仍只出现在点云里。
    const double x0 = sim.current().ghosts.front().pose.x;
    for (int i = 0; i < 25; ++i) {
        clock.advance(0.02);
        sim.stepIfDue(clock.monotonicSeconds());
    }
    check(std::abs(sim.current().ghosts.front().pose.x - x0) < 1e-12,
          "注入目标不被物理引擎推进（静止）");

    // ---- 回看：注入前那一帧看不到它 ----
    const adasim::domain::FrameId latest = sim.latestFrame();
    check(latest > 40, "已录到足够帧数");
    check(sim.previewSeek(2), "previewSeek 到注入前成功");
    check(sim.ghostCount() == 0, "回看到注入前，注入目标消失");

    // 回看不销毁历史：最新帧号不能变。
    check(sim.bufferLatest() == latest, "previewSeek 不截断历史");
    check(sim.latestFrame() < latest, "回看位置与已录末帧是两回事");
    check(buffer.at(latest).has_value(), "历史帧仍在缓冲中");

    // 回到注入之后，它又出现了——说明记录本身带着注入。
    check(sim.previewSeek(latest), "previewSeek 回到注入后成功");
    check(sim.ghostCount() == 1, "回看到注入后，目标重新出现");

    // ---- 从历史位置继续跑 = 分支，这时才截断 ----
    check(sim.previewSeek(latest - 20), "回看到中途成功");
    sim.start();
    check(sim.latestFrame() == latest - 20, "start() 在历史位置截断时间线");
    sim.pause();

    // ---- 清除 ----
    // 暂停状态下 stepIfDue 不会推进（这是对的），这里用 stepOnce 强制出帧，
    // 才能验证"清除之后点云里确实没有点了"。
    sim.clearGhosts();
    check(sim.ghostCount() == 0, "清除后 ghost 计数为 0");
    for (int i = 0; i < 8; ++i) {
        clock.advance(0.02);
        sim.stepOnce();
    }
    check(transport.lastCloud.pointCount() == 0, "清除后点云回到 0 点");

    std::printf("\n%s（失败 %d 项）\n", g_failed == 0 ? "全部通过" : "存在失败", g_failed);
    return g_failed == 0 ? 0 : 1;
}
