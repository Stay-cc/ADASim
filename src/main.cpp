#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <algorithm>
#include <cstdio>
#include <string>

#include "application/ExperimentRunner.h"
#include "application/ScenarioService.h"
#include "application/SimController.h"
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
    QApplication qtApp(argc, argv);
    QApplication::setApplicationName(QStringLiteral("ADASim"));

    // ---- L4 基础设施：全部是端口的具体实现 ----
    adasim::infra::SteadyClock clock;
    adasim::infra::ConsoleLogger logger;
    adasim::infra::JsonScenarioRepository scenarios(
        QDir(QCoreApplication::applicationDirPath())
            .absoluteFilePath(QStringLiteral("../scenarios"))
            .toStdString());
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
    window.setLinkProbe([&gateway]() {
        if (!gateway.listening()) {
            return QStringLiteral("通信未启动");
        }
        return gateway.linkAlive() ? QStringLiteral("算法进程 ● 在线")
                                   : QStringLiteral("算法进程 ○ 等待连接");
    });

    if (gateway.listen()) {
        logger.log(adasim::domain::LogLevel::Info,
                   "通信网关已监听 127.0.0.1:" + std::to_string(gateway.boundTcpPort()));
    } else {
        // 端口被占用不阻断仿真：单机调试时同时开两个实例很常见。
        logger.log(adasim::domain::LogLevel::Warn,
                   "通信网关监听失败：" + gateway.errorString().toStdString() +
                       "，仿真功能不受影响");
    }

    window.show();

    // 命令行参数：
    //   adasim [场景文件]                正常启动
    //   adasim --shot <图片路径> [场景]   自动跑 2.5 s 后截图退出
    //   --shot-delay <毫秒>              调整截图前的运行时长
    // 截图模式不是为了测试方便才加的：论文与答辩需要的界面配图
    // 必须可重复生成，手按截图键得到的图每次都不一样。
    // 延时之所以可配，是因为"算法进程在线"这类联调状态需要跑够时间
    // 才能出现，固定 2.5 s 会在客户端还没连上时就截图。
    QString scenarioPath;
    QString shotPath;
    int shotDelayMs = 2500;

    const QStringList args = QCoreApplication::arguments();
    for (int i = 1; i < args.size(); ++i) {
        const QString& a = args.at(i);
        if (a == QLatin1String("--shot") && i + 1 < args.size()) {
            shotPath = args.at(++i);
        } else if (a == QLatin1String("--shot-delay") && i + 1 < args.size()) {
            shotDelayMs = std::max(200, args.at(++i).toInt());
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

    return qtApp.exec();
}
