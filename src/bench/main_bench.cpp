// 点云压测工具：把"十万点能不能实时"这件事量出来，而不是估出来。
//
// 它只做三件事，每件都对应论文里一个要报告的数字：
//   1. LidarModel::sample 的耗时        —— 点云是谁生成的，生成本身要多久
//   2. QtRenderer::rasterize 的耗时     —— 渲染上限在哪（分全帧/点云段）
//   3. UDP 分片发送的耗时与带宽         —— 十万点上线要多少包、多少兆
//
// 场景是合成的密排车阵（见 tools/gen_dense_scenario.py），因为正常交通
// 在 50 m 量程内根本产生不出十万点——这一点本身就是压测的结论之一，
// 报告里必须写清楚，否则"十万点 ≥60 FPS"会被误解为"路上随便就能遇到"。
//
// 用法：
//   adasim_bench                       默认扫描 6 档点云规模
//   adasim_bench --iters 200           提高采样次数（噪声大的机器）
//   adasim_bench --width 1920 --height 1080
//   adasim_bench --no-udp              跳过网络段（端口被占用时）

#include <QCoreApplication>
#include <QElapsedTimer>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "domain/model/Packet.h"
#include "domain/model/WorldSnapshot.h"
#include "domain/model/WorldState.h"
#include "domain/sensor/LidarModel.h"
#include "infrastructure/net/TcpUdpGateway.h"
#include "infrastructure/render/QtRenderer.h"

namespace {

constexpr double kCarLength = 4.5;
constexpr double kCarWidth = 1.9;
constexpr double kCarPerimeter = 2.0 * (kCarLength + kCarWidth);
constexpr int kLanes = 24;          // 与压测场景一致：自车所在车道留空
constexpr double kLaneWidth = 3.5;
constexpr double kSpacing = 5.4;    // 车头到车头，m
constexpr double kStartX = 12.0;

/// 构造密排车阵。自车在原点、航向 +x，y=0 车道留空以便自车能往前开。
adasim::domain::WorldState makeWorld(int cars, double speed) {
    using namespace adasim::domain;
    WorldState state;
    state.egoSpeed = speed;

    const int columns = std::max(1, cars / kLanes);
    const int half = kLanes / 2;
    std::uint32_t id = 1;
    for (int col = 0; col < columns; ++col) {
        const double x = kStartX + col * kSpacing;
        for (int i = 1; i <= half; ++i) {
            for (int sign = -1; sign <= 1; sign += 2) {
                EntityState e;
                e.id = id++;
                e.kind = EntityKind::Car;
                e.pose.x = x;
                e.pose.y = sign * i * kLaneWidth;
                e.speed = speed;
                e.length = kCarLength;
                e.width = kCarWidth;
                state.obstacles.push_back(e);
            }
        }
    }
    return state;
}

/// 真实车流基线：本车道内 n 辆车，间隔 25 m。这档存在的意义是给出
/// "正常道路"的参照点——压测数字若没有对照，读者无法判断 255 FPS
/// 究竟算快还是算慢。
adasim::domain::WorldState makeFollowingWorld(int cars, double speed) {
    using namespace adasim::domain;
    WorldState state;
    state.egoSpeed = speed;
    for (int i = 0; i < cars; ++i) {
        EntityState e;
        e.id = static_cast<std::uint32_t>(i + 1);
        e.kind = EntityKind::Car;
        e.pose.x = 25.0 + i * 25.0;
        e.speed = speed * 0.5;
        e.length = kCarLength;
        e.width = kCarWidth;
        state.obstacles.push_back(e);
    }
    return state;
}

double percentile(std::vector<double> v, double q) {
    if (v.empty()) {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    const std::size_t idx = static_cast<std::size_t>(q * static_cast<double>(v.size() - 1));
    return v[idx];
}

struct Timing {
    double median{0.0};
    double p95{0.0};
    double p99{0.0};   ///< 开题承诺的指标是 P99 ≤ 16.7 ms，只报 p95 不够
};

Timing summarize(const std::vector<double>& samples) {
    Timing t;
    t.median = percentile(samples, 0.50);
    t.p95 = percentile(samples, 0.95);
    t.p99 = percentile(samples, 0.99);
    return t;
}

struct Row {
    int cars{0};
    double density{0.0};
    std::size_t points{0};
    Timing generate;
    Timing frame;
    Timing cloud;
};

}  // namespace

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);

    int iterations = 100;
    int width = 1280;
    int height = 720;
    bool withUdp = true;

    const QStringList args = QCoreApplication::arguments();
    for (int i = 1; i < args.size(); ++i) {
        const QString& a = args.at(i);
        if (a == QLatin1String("--iters") && i + 1 < args.size()) {
            iterations = std::max(10, args.at(++i).toInt());
        } else if (a == QLatin1String("--width") && i + 1 < args.size()) {
            width = std::max(320, args.at(++i).toInt());
        } else if (a == QLatin1String("--height") && i + 1 < args.size()) {
            height = std::max(240, args.at(++i).toInt());
        } else if (a == QLatin1String("--no-udp")) {
            withUdp = false;
        }
    }

    adasim::infra::QtRenderer renderer;
    adasim::domain::Viewport vp;
    vp.widthPx = width;
    vp.heightPx = height;
    vp.metersPerPixel = 0.1;   // 与实际界面默认缩放一致
    vp.centerX = 60.0;         // 把车阵中部放进视口
    vp.centerY = 0.0;
    renderer.setViewport(vp);

    // 扫描用的规模档位：从"真实车流"到"合成上界"，最后两档专门找拐点。
    struct Config {
        int cars;
        double density;
        bool grid;
        const char* label;
    };
    const Config configs[] = {
        {2, 1.5, false, "基准：2 车跟驰（默认密度）"},
        {2, 10.0, false, "基准：2 车跟驰（0.1 m 采样）"},
        {192, 5.0, true, "中密度车阵：192 车"},
        {384, 10.0, true, "高密度车阵：384 车"},
        {792, 10.0, true, "压测题设：792 车"},
        {792, 20.0, true, "两倍密度"},
        {792, 40.0, true, "四倍密度（找拐点）"},
    };

    std::vector<Row> rows;
    rows.reserve(sizeof(configs) / sizeof(configs[0]));

    std::printf("视口            : %dx%d @ %.2f m/px（可见约 %.0f x %.0f m）\n",
                width, height, vp.metersPerPixel, width * vp.metersPerPixel,
                height * vp.metersPerPixel);
    std::printf("采样次数        : %d（取中位数与 p95）\n\n", iterations);

    std::shared_ptr<adasim::domain::WorldSnapshot> snapshot;

    for (const auto& cfg : configs) {
        adasim::domain::LidarParams lidar;
        lidar.range = 250.0;
        lidar.pointsPerMeter = cfg.density;

        const auto world = cfg.grid ? makeWorld(cfg.cars, 8.0)
                                    : makeFollowingWorld(cfg.cars, 8.0);

        auto snap = std::make_shared<adasim::domain::WorldSnapshot>();
        adasim::domain::LidarModel::sample(world, lidar, *snap);

        std::vector<double> genSamples;
        genSamples.reserve(static_cast<std::size_t>(iterations));
        for (int i = 0; i < iterations; ++i) {
            QElapsedTimer t;
            t.start();
            adasim::domain::LidarModel::sample(world, lidar, *snap);
            genSamples.push_back(static_cast<double>(t.nsecsElapsed()) / 1.0e6);
        }

        // 渲染：submit + requestFrame。用 shared_ptr<const> 传递，
        // 与实际链路一致（控制器构造快照后交给渲染端口）。
        std::shared_ptr<const adasim::domain::WorldSnapshot> frozen = snap;
        std::vector<double> frameSamples;
        std::vector<double> cloudSamples;
        frameSamples.reserve(static_cast<std::size_t>(iterations));
        cloudSamples.reserve(static_cast<std::size_t>(iterations));
        renderer.submit(frozen);
        renderer.requestFrame();  // 预热：首帧要分配画布
        for (int i = 0; i < iterations; ++i) {
            renderer.requestFrame();
            frameSamples.push_back(renderer.lastFrameMillis());
            cloudSamples.push_back(renderer.lastCloudMillis());
        }

        Row row;
        row.cars = static_cast<int>(world.obstacles.size());
        row.density = cfg.density;
        row.points = snap->pointCount();
        row.generate = summarize(genSamples);
        row.frame = summarize(frameSamples);
        row.cloud = summarize(cloudSamples);
        rows.push_back(row);

        std::printf("[%s]\n", cfg.label);
        std::printf("  车数 %d  密度 %.1f 点/米  点数 %zu\n", row.cars, row.density, row.points);
        std::printf("  点云生成 : 中位 %.3f ms  p95 %.3f ms\n", row.generate.median, row.generate.p95);
        std::printf("  整帧栅格 : 中位 %.3f ms  p95 %.3f ms  p99 %.3f ms（其中点云段 %.3f / %.3f ms）\n",
                    row.frame.median, row.frame.p95, row.frame.p99, row.cloud.median, row.cloud.p95);
        std::printf("  等效帧率 : %.0f FPS（按 p95 折算）  %s\n\n",
                    1000.0 / std::max(0.001, row.frame.p95),
                    row.frame.p99 <= 16.7 ? "[P99 <= 16.7 ms 达标]" : "[P99 超 16.7 ms]");

        if (row.points >= 100000 && snapshot == nullptr) {
            snapshot = snap;  // 留一份十万点快照给后面的网络段
        }
    }

    // ---- 汇总表 ----
    // 表头用英文而不是中文：printf 的宽度按字节算，一个汉字三字节，
    // 用 %8s 对齐只会让列越错越远。数字列的对齐比语言一致性更重要。
    std::printf("=== 汇总（%dx%d，%d 次采样）===\n", width, height, iterations);
    std::printf("%6s %8s %10s %18s %22s %6s\n",
                "cars", "p/m", "points", "gen med/p95", "frame med/p95/p99", "fps");
    for (const auto& r : rows) {
        std::printf("%6d %8.1f %10zu %8.3f/%6.3f %8.3f/%6.3f/%6.3f %6.0f\n",
                    r.cars, r.density, r.points, r.generate.median, r.generate.p95,
                    r.frame.median, r.frame.p95, r.frame.p99,
                    1000.0 / std::max(0.001, r.frame.p95));
    }

    if (snapshot != nullptr) {
        const std::size_t pts = snapshot->pointCount();
        const double memMb = static_cast<double>(pts) * 3.0 * sizeof(float) / (1024.0 * 1024.0);
        std::printf("\n=== 十万点档的附加测量 ===\n");
        std::printf("单帧点云内存    : %.2f MB（3 个 float 数组）\n", memMb);
        std::printf("单帧报文字节    : %.2f MB（每点 12 字节，未压缩）\n",
                    static_cast<double>(pts) * 12.0 / (1024.0 * 1024.0));

        // 1080p：只跑十万点这一档，用来看分辨率对光栅化的影响。
        adasim::domain::Viewport hd = vp;
        hd.widthPx = 1920;
        hd.heightPx = 1080;
        renderer.setViewport(hd);
        renderer.submit(snapshot);
        renderer.requestFrame();
        std::vector<double> hdSamples;
        for (int i = 0; i < iterations; ++i) {
            renderer.requestFrame();
            hdSamples.push_back(renderer.lastFrameMillis());
        }
        const Timing hdTiming = summarize(hdSamples);
        std::printf("1920x1080 整帧  : 中位 %.3f ms  p95 %.3f ms  p99 %.3f ms（%.0f FPS）\n",
                    hdTiming.median, hdTiming.p95, hdTiming.p99,
                    1000.0 / std::max(0.001, hdTiming.p95));

        // ---- UDP 分片：对比"按 UDP 上限切"与"按 MTU 切" ----
        //
        // 两档的区别不只是包数：24 KB 的报文在环回上能整包过，出了网卡
        // 就会被 IP 层切成 17 个分片，任一分片丢失整包作废。这里两档都
        // 实测一遍，用来支撑"为什么默认值从 2000 点改成 120 点"。
        if (withUdp) {
            struct UdpResult {
                int chunk{0};
                int chunks{0};
                Timing timing;
                quint64 dropped{0};
                bool ok{false};
            };

            const auto measure = [&](int chunkPoints) {
                UdpResult r;
                adasim::infra::GatewayConfig cfg;
                cfg.cloudChunkPoints = chunkPoints;
                {
                    adasim::infra::TcpUdpGateway gateway(cfg);
                    if (!gateway.listen()) {
                        return r;
                    }
                    adasim::domain::PointCloudPacket packet;
                    packet.px = snapshot->px;
                    packet.py = snapshot->py;
                    packet.intensity = snapshot->intensity;

                    std::vector<double> samples;
                    samples.reserve(20);
                    for (int i = 0; i < 20; ++i) {
                        packet.frameId = static_cast<adasim::domain::FrameId>(i + 1);
                        QElapsedTimer t;
                        t.start();
                        gateway.publishCloud(packet);
                        samples.push_back(static_cast<double>(t.nsecsElapsed()) / 1.0e6);
                    }
                    r.chunk = chunkPoints;
                    r.chunks = static_cast<int>((pts + chunkPoints - 1) / chunkPoints);
                    r.timing = summarize(samples);
                    r.dropped = gateway.droppedCloudChunks();
                    r.ok = true;
                }
                return r;   // 网关在作用域结束时析构，端口即刻释放
            };

            const double bytes = static_cast<double>(pts) * 12.0;
            std::printf("\nUDP 分片（每帧 %.2f MB，10 Hz 即 %.0f Mbit/s）\n",
                        bytes / (1024.0 * 1024.0), bytes * 8.0 * 10.0 / 1.0e6);
            std::printf("%12s %10s %14s %18s\n", "每片点数", "片数", "片大小", "发送 中位/p95");
            for (const int chunk : {2000, 120}) {
                const UdpResult r = measure(chunk);
                if (!r.ok) {
                    std::printf("端口被占用，UDP 段跳过\n");
                    break;
                }
                const int payload = 24 + chunk * 12;
                std::printf("%12d %10d %11d B %8.3f/%6.3f ms%s\n", r.chunk, r.chunks, payload,
                            r.timing.median, r.timing.p95,
                            payload > 1472 ? "   <- 超 MTU，走 IP 分片" : "   <- 单包不超 MTU");
            }
        }
    }

    return 0;
}
