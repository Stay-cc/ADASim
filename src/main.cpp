#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <functional>
#include <string>

#include "application/ExperimentRunner.h"
#include "application/ScenarioService.h"
#include "application/SimController.h"
#include "domain/model/ForwardGap.h"
#include "domain/simulation/PhysicsEngine.h"
#include "domain/timeline/TimelineBuffer.h"
#include "infrastructure/log/ConsoleLogger.h"
#include "infrastructure/net/TcpUdpGateway.h"
#include "infrastructure/persist/JsonScenarioRepository.h"
#include "infrastructure/render/QtRenderer.h"
#include "infrastructure/time/SteadyClock.h"
#include "presentation/MainWindow.h"

/// 装配根。
///
/// 整个程序里只有这个文件同时 include 了 application 与 infrastructure。
/// 其余目标要么只认识接口（domain），要么只认识编排对象（app），
/// 所以把 Qt 换成别的渲染后端、把 TCP 换成共享内存，
/// 改动都被限制在这一个函数体内。
int main(int argc, char** argv) {
    // 冷启动的起点必须在这里取：放进 QApplication 之后就漏掉了 Qt 自身的
    // 平台初始化（Windows 上就是几百毫秒），而这恰恰是对比 CARLA 类重型
    // 平台时最关心的一项。用 steady_clock 而不是 wall clock——后者会跳。
    const auto coldStartAt = std::chrono::steady_clock::now();

    QApplication qtApp(argc, argv);
    QApplication::setApplicationName(QStringLiteral("ADASim"));

    // ---- L4 基础设施：全部是端口的具体实现 ----
    adasim::infra::SteadyClock clock;
    adasim::infra::ConsoleLogger logger;
    // 场景目录相对可执行文件上三级（bin → build → 项目根 → scenarios）。
    // 早先写成 ../scenarios，从 build/*/bin 运行时指向 build/*/scenarios，
    // 目录不存在，导致 listTemplates() 返回空——命令行传绝对路径时被掩盖了，
    // 场景选择页一依赖它才暴露。找不到时退回相对当前工作目录的 scenarios，
    // 兼容"在项目根跑 exe"与"换部署位置"两种情形。
    const QString exeDir = QCoreApplication::applicationDirPath();
    QString scenarioRoot = QDir(exeDir).absoluteFilePath(QStringLiteral("../../../scenarios"));
    if (!QDir(scenarioRoot).exists()) {
        scenarioRoot = QDir(exeDir).absoluteFilePath(QStringLiteral("../scenarios"));
    }
    if (!QDir(scenarioRoot).exists()) {
        scenarioRoot = QDir::current().absoluteFilePath(QStringLiteral("scenarios"));
    }
    adasim::infra::JsonScenarioRepository scenarios(scenarioRoot.toStdString());
    adasim::infra::TcpUdpGateway gateway;
    adasim::infra::QtRenderer renderer;

    // ---- L3 领域：纯计算，不知道外面有 Qt 与网络 ----
    adasim::domain::PhysicsEngine physics;
    // 1500 帧 × 20 ms = 30 s 窗口；每 25 帧（0.5 s）一份完整快照。
    adasim::domain::TimelineBuffer buffer(1500, 25);

    // ---- L2 应用：只拿端口引用，不拿实现类型 ----
    adasim::app::SimController sim(physics, buffer, gateway, clock);
    adasim::app::ScenarioService scenarioService(scenarios);
    adasim::app::ExperimentRunner runner(physics, clock);

    // ---- L1 表现层 ----
    adasim::ui::MainWindow window(sim, scenarioService, runner, renderer);
    window.setAlgoOnlineProbe([&gateway]() { return gateway.linkAlive(); });

    if (gateway.listen()) {
        logger.log(adasim::domain::LogLevel::Info,
                   "通信网关已监听 127.0.0.1:" + std::to_string(gateway.boundTcpPort()));
    } else {
        // 端口被占用不阻断仿真：单机调试时同时开两个实例很常见。
        // 但必须让用户看见——否则新开的这个窗口画面一切正常、算法却永远连不上，
        // 表现为"HUD 一直离线"，而真正占着端口的那个实例可能已经被最小化了。
        // 端口号取配置默认值而不是 boundTcpPort()：绑定失败时它返回 0，
        // 提示里就会变成"端口 0 被占用"，等于没说。
        const QString note = QStringLiteral("端口 %1 被占用")
                                 .arg(adasim::infra::GatewayConfig{}.tcpPort);
        window.setGatewayInfo(false, note);
        logger.log(adasim::domain::LogLevel::Warn,
                   "通信网关监听失败：" + gateway.errorString().toStdString() + "。" +
                       "算法进程会连到占用端口的那个实例上，本窗口收不到指令；"
                       "关掉旧实例或改端口后再试");
    }

    window.show();

    // 冷启动指标：从进程入口到首帧贴图的墙钟时间，直接打一行。
    // 不做成"每次运行都显示在 HUD 上"——它只对首次有意义，
    // 常驻反而占掉 HUD 一行。
    QObject::connect(&window, &adasim::ui::MainWindow::firstFrameReady, &window,
                     [coldStartAt]() {
                         const double ms =
                             std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - coldStartAt)
                                 .count();
                         std::fprintf(stdout, "首帧就绪: %.0f ms\n", ms);
                         std::fflush(stdout);
                     });

    // 命令行参数：
    //   adasim [场景文件]                正常启动
    //   adasim --shot <图片路径> [场景]   自动跑 2.5 s 后截图退出
    //   --shot-delay <毫秒>              调整截图前的运行时长
    //   adasim --record <目录> [场景]     自动跑一段并连续截图，用于录 Demo
    //   --frames <张> / --interval <毫秒> 控制录制的张数与间隔
    // 截图模式不是为了测试方便才加的：论文与答辩需要的界面配图
    // 必须可重复生成，手按截图键得到的图每次都不一样。
    // 延时之所以可配，是因为"算法进程在线"这类联调状态需要跑够时间
    // 才能出现，固定 2.5 s 会在客户端还没连上时就截图。
    //
    // --record 是在 --shot 之上加的一条"连续帧"通路：单张截图说明不了任何
    // 动态过程，而 Demo 要展示的恰恰是过程（车在动、前车在刹、断链后自车
    // 自己减速停车）。一次运行里连续取帧，才能保证这些帧属于同一次仿真。
    QString scenarioPath;
    QString shotPath;
    int shotDelayMs = 2500;
    QString recordDir;
    int recordFrames = 8;
    int recordIntervalMs = 1000;
    QString metricsPath;
    // 无人值守时没法用鼠标右键，但"注入 → 点云 → 算法 → 制动"这条链路
    // 必须能被自动验证与出图。--inject 就是给这条路留的入口，
    // 与右键走的是同一个 SimController::injectGhost。
    QStringList injectSpecs;
    int injectDelayMs = 0;
    bool showPicker = false;

    const QStringList args = QCoreApplication::arguments();
    for (int i = 1; i < args.size(); ++i) {
        const QString& a = args.at(i);
        if (a == QLatin1String("--shot") && i + 1 < args.size()) {
            shotPath = args.at(++i);
        } else if (a == QLatin1String("--shot-delay") && i + 1 < args.size()) {
            shotDelayMs = std::max(200, args.at(++i).toInt());
        } else if (a == QLatin1String("--record") && i + 1 < args.size()) {
            recordDir = args.at(++i);
        } else if (a == QLatin1String("--frames") && i + 1 < args.size()) {
            recordFrames = std::max(1, args.at(++i).toInt());
        } else if (a == QLatin1String("--interval") && i + 1 < args.size()) {
            recordIntervalMs = std::max(100, args.at(++i).toInt());
        } else if (a == QLatin1String("--metrics") && i + 1 < args.size()) {
            metricsPath = args.at(++i);
        } else if (a == QLatin1String("--inject") && i + 1 < args.size()) {
            injectSpecs.push_back(args.at(++i));
        } else if (a == QLatin1String("--inject-delay") && i + 1 < args.size()) {
            injectDelayMs = std::max(0, args.at(++i).toInt());
        } else if (a == QLatin1String("--picker")) {
            showPicker = true;
        } else if (!a.startsWith(QLatin1String("--"))) {
            scenarioPath = a;
        }
    }

    if (!scenarioPath.isEmpty()) {
        // 命令行给的相对路径按"当前工作目录"解析，而不是按场景目录。
        // 仓储的 resolve() 对相对路径会拼上场景根目录，两者语义不同；
        // 若沿用相对路径，`adasim scenarios/x.json` 会被拼成
        // <场景目录>/scenarios/x.json 而加载失败，界面则静默退回默认场景。
        window.loadScenarioFile(QFileInfo(scenarioPath).absoluteFilePath());
    } else if (showPicker) {
        // 直接跳到场景选择页：截图/测试场景选择页布局用。
        window.showScenarioPicker();
    }

    if (!injectSpecs.isEmpty()) {
        // 在指定延时后统一注入：放在 run 起来之后，注入才真的是"运行时"发生，
        // 而不是场景初始化的一部分（后者会让前几帧的点云里已经有它）。
        const QStringList specs = injectSpecs;
        auto apply = [&sim, specs]() {
            for (const QString& spec : specs) {
                const QStringList parts = spec.split(QLatin1Char(','));
                if (parts.size() < 2) {
                    std::fprintf(stderr, "注入参数格式应为 x,y[,car|truck|ped]：%s\n",
                                 spec.toLocal8Bit().constData());
                    continue;
                }
                auto kind = adasim::domain::EntityKind::Car;
                if (parts.size() >= 3) {
                    const QString k = parts.at(2).trimmed().toLower();
                    if (k == QLatin1String("truck")) {
                        kind = adasim::domain::EntityKind::Truck;
                    } else if (k == QLatin1String("ped") || k == QLatin1String("pedestrian")) {
                        kind = adasim::domain::EntityKind::Pedestrian;
                    }
                }
                const std::uint32_t id = sim.injectGhost(
                    parts.at(0).toDouble(), parts.at(1).toDouble(),
                    sim.current().ego.theta, kind);
                std::fprintf(stdout, "已注入目标 #%u @ (%s)\n", id,
                             spec.toLocal8Bit().constData());
            }
        };
        if (injectDelayMs <= 0) {
            apply();
        } else {
            QTimer::singleShot(injectDelayMs, &window, apply);
        }
    }

    if (!shotPath.isEmpty()) {
        sim.start();
        QTimer::singleShot(shotDelayMs, &window, [&window, shotPath]() {
            // 渲染在独立线程里完成，图像通过信号回传到控件。
            // grab() 抓的是控件当前的位图，若恰好在两帧之间就会抓到旧帧，
            // 表现为截图与状态栏数据不一致（甚至抓到初始空场景）。
            // 先把挂起的事件处理完，让最后一张图落到控件上再截。
            QCoreApplication::processEvents();
            const bool ok = window.grab().save(shotPath, "PNG");
            std::fprintf(ok ? stdout : stderr, "%s: %s\n",
                         ok ? "已保存截图" : "截图保存失败",
                         shotPath.toLocal8Bit().constData());
            QCoreApplication::quit();
        });
    }

    // 录制伴随的时序采样：截图只说明"那一刻长什么样"，车在加速还是已经刹停、
    // 断链发生在第几秒，得靠数值。采样独立于抓图节拍（固定 100 ms），
    // 否则 8 张图只能给出 8 个点的曲线。
    std::ofstream metrics;
    if (!metricsPath.isEmpty()) {
        // 与 --record 同目录时，录制目录可能还没建，这里先补上。
        QDir().mkpath(QFileInfo(metricsPath).absolutePath());
        metrics.open(metricsPath.toStdString(), std::ios::out | std::ios::trunc);
        if (!metrics) {
            std::fprintf(stderr, "无法写入时序文件: %s\n",
                         metricsPath.toLocal8Bit().constData());
        } else {
            // ghosts 单列一列：注入的目标不进真值，gap 那列看不到它们，
            // 没有这一列就无法从时序上判断"车停下是因为有注入目标"。
            metrics << "t,x,speed,gap,standby,link,ghosts,frame\n";
            auto* sampler = new QTimer(&qtApp);
            QObject::connect(sampler, &QTimer::timeout, &window,
                             [&sim, &gateway, &metrics]() {
                const adasim::domain::WorldState& s = sim.current();
                const adasim::domain::ForwardGap gap =
                    adasim::domain::nearestForwardTarget(s);
                // link 与 standby 是两回事：link 表示"算法进程在不在"，
                // standby 表示"这一帧兜底律是否正在制动"。车距还有余量时
                // 断链也不会制动，两者混用会把"离线但没刹"读成"在线"。
                metrics << s.simTime << ',' << s.ego.x << ',' << s.egoSpeed << ','
                        << (gap.valid ? gap.bumperGap : -1.0) << ','
                        << (sim.standbyGuardActive() ? 1 : 0) << ','
                        << (gateway.linkAlive() ? 1 : 0) << ','
                        << sim.ghostCount() << ',' << s.frameId
                        << '\n';
            });
            sampler->start(100);
        }
    }

    if (!recordDir.isEmpty()) {
        // 目录不存在就建：调用方（tools/demo.py）只负责传路径，不该先 mkdir。
        if (!QDir().mkpath(recordDir)) {
            std::fprintf(stderr, "无法创建录制目录: %s\n",
                         recordDir.toLocal8Bit().constData());
            return 2;
        }

        sim.start();

        // 用自递归的 singleShot 而不是周期 timer：抓图本身要几十毫秒，
        // 周期 timer 会在慢帧上排队，越跑越偏，最后两张时间戳挨在一起。
        //
        // capture 存成 static 而不是 shared_ptr 自引用：自引用会形成环，
        // 而"执行中的 std::function 销毁自己"是未定义行为，不能靠置空来破环。
        static int shotIndex = 0;
        static std::function<void()> capture;
        capture = [&window, recordDir, recordFrames, recordIntervalMs]() {
            QCoreApplication::processEvents();
            const QString path =
                QDir(recordDir).filePath(QStringLiteral("frame_%1.png")
                                             .arg(shotIndex, 2, 10, QLatin1Char('0')));
            const bool ok = window.grab().save(path, "PNG");
            std::fprintf(ok ? stdout : stderr, "%s: %s\n",
                         ok ? "已保存帧" : "帧保存失败",
                         path.toLocal8Bit().constData());
            std::fflush(stdout);
            if (++shotIndex >= recordFrames) {
                QCoreApplication::quit();
                return;
            }
            QTimer::singleShot(recordIntervalMs, capture);
        };
        QTimer::singleShot(recordIntervalMs, capture);
    }

    return qtApp.exec();
}
