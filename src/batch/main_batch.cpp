// 批量评估进程：失效搜索的"评价函数"这一侧的端点。
//
// 协议是行式 JSON，走 stdin/stdout 而不是 TCP：
//   请求  {"path": "场景文件", "noise": 0.4, "seed": 1, "duration": 20, "closedLoop": true}
//   响应  {"ok": true, "collided": false, "minTtc": 1.23, ...}
//
// 为什么不用既有 TCP 网关：那是实时链路的通道，一套心跳、一帧一频的语义
// 都为在线仿真设计。离线评估是"请求—应答"式的短事务，行式管道更贴切，
// 也不会让 AI 进程误以为自己在跟一台正在跑的仿真器说话。
//
// 为什么常驻而不是每次起一个进程：一次评估约 3~5 ms，而进程启动 + Qt 初始化
// 要 100 ms 以上。100 次评估若每次都起进程，光启动开销就 10 s 起步，
// 会让"100 次预算"这个前提失去意义。常驻进程让预算真正花在仿真上。
//
// 场景文件由调用方写好、本进程只读：好处是每一轮评估跑了什么都有据可查，
// 事后能把"最优参数"原样复现出来。

#include <QByteArray>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "application/ExperimentRunner.h"
#include "application/ScenarioService.h"
#include "domain/simulation/PhysicsEngine.h"
#include "infrastructure/persist/JsonScenarioRepository.h"
#include "infrastructure/time/SteadyClock.h"

namespace {

void writeLine(const QJsonObject& obj) {
    const QByteArray bytes = QJsonDocument(obj).toJson(QJsonDocument::Compact);
    std::fwrite(bytes.constData(), 1, static_cast<std::size_t>(bytes.size()), stdout);
    std::fputc('\n', stdout);
    std::fflush(stdout);   // 必须立刻刷出：调用方按行阻塞读取
}

QJsonObject failure(const QString& message) {
    QJsonObject obj;
    obj.insert(QStringLiteral("ok"), false);
    obj.insert(QStringLiteral("error"), message);
    return obj;
}

}  // namespace

int main(int argc, char** argv) {
    std::string rootDir = ".";
    std::string oncePath;
    bool once = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--root" && i + 1 < argc) {
            rootDir = argv[++i];
        } else if (arg == "--once" && i + 1 < argc) {
            once = true;
            oncePath = argv[++i];
        }
    }

    adasim::infra::JsonScenarioRepository repository(rootDir);
    adasim::app::ScenarioService scenarios(repository);
    adasim::domain::PhysicsEngine physics;
    adasim::infra::SteadyClock clock;
    adasim::app::ExperimentRunner runner(physics, clock);

    auto evaluate = [&](const std::string& path, double noise, std::uint32_t seed,
                        double duration, bool closedLoop) -> QJsonObject {
        const auto tmpl = scenarios.load(path);
        if (!tmpl.has_value()) {
            const auto raw = repository.load(path);
            if (!raw.has_value()) {
                return failure(QString::fromStdString(repository.lastError()));
            }
            QJsonArray issues;
            for (const auto& issue : scenarios.validate(*raw)) {
                issues.append(QString::fromStdString(issue));
            }
            QJsonObject obj = failure(QStringLiteral("场景校验失败"));
            obj.insert(QStringLiteral("issues"), issues);
            return obj;
        }

        adasim::app::EvaluationConfig cfg;
        cfg.durationSeconds = duration > 0.0 ? duration : tmpl->durationSeconds;
        cfg.noise = noise;
        cfg.seed = seed;
        cfg.closedLoop = closedLoop;

        const adasim::domain::WorldState initial = scenarios.buildInitialState(*tmpl);
        const adasim::app::ExperimentMetrics m = runner.run(initial, cfg);

        QJsonObject obj;
        obj.insert(QStringLiteral("ok"), true);
        obj.insert(QStringLiteral("collided"), m.collided);
        // 未接近过任何目标时 minTtc/minGap 保持初值 1e9；换成 -1 明确表示"不适用"，
        // 免得下游把 1e9 当成真实数值参与拟合。
        obj.insert(QStringLiteral("minTtc"), m.minTtc >= 1e8 ? -1.0 : m.minTtc);
        obj.insert(QStringLiteral("minGap"), m.minGap >= 1e8 ? -1.0 : m.minGap);
        obj.insert(QStringLiteral("maxAbsAccel"), m.maxAbsAccel);
        obj.insert(QStringLiteral("impactSpeed"), m.impactSpeed);
        obj.insert(QStringLiteral("travelled"), m.travelled);
        obj.insert(QStringLiteral("steps"), static_cast<qint64>(m.steps));
        obj.insert(QStringLiteral("wallMs"), m.wallSeconds * 1000.0);
        return obj;
    };

    if (once) {
        writeLine(evaluate(oncePath, 0.0, 1, 0.0, true));
        return 0;
    }

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        QJsonParseError error{};
        const QJsonDocument doc =
            QJsonDocument::fromJson(QByteArray::fromStdString(line), &error);
        if (error.error != QJsonParseError::NoError || !doc.isObject()) {
            writeLine(failure(QStringLiteral("请求不是合法 JSON: ") + error.errorString()));
            continue;
        }

        const QJsonObject request = doc.object();
        const std::string path = request.value(QStringLiteral("path")).toString().toStdString();
        if (path.empty()) {
            writeLine(failure(QStringLiteral("请求缺少 path 字段")));
            continue;
        }

        const double noise = request.value(QStringLiteral("noise")).toDouble(0.0);
        const double duration = request.value(QStringLiteral("duration")).toDouble(0.0);
        const bool closedLoop = request.value(QStringLiteral("closedLoop")).toBool(true);
        const auto seedValue = request.value(QStringLiteral("seed")).toVariant().toULongLong();
        const auto seed = static_cast<std::uint32_t>(seedValue == 0 ? 1 : seedValue);

        writeLine(evaluate(path, noise, seed, duration, closedLoop));
    }

    return 0;
}
