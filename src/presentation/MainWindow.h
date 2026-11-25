#pragma once

#include <QMainWindow>
#include <QString>
#include <functional>

#include "domain/model/Viewport.h"

class QLabel;
class QSlider;
class QComboBox;
class QTimer;
class QAction;
class QStackedWidget;
class QToolBar;

namespace adasim::app {
class SimController;
class ScenarioService;
class ExperimentRunner;
}  // namespace adasim::app

namespace adasim::infra {
class QtRenderer;
}

namespace adasim::ui {

class ViewportWidget;
class HomeWidget;
class ScenarioPickerWidget;

/// 主窗口：只做编排与显示，不实现任何仿真逻辑。
///
/// 它对仿真只有四个动作：播放/暂停、单步、回溯、跑实验。
/// 这四件事全部转交给 SimController，界面不持有物理引擎、不碰时间轴。
class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(app::SimController& sim,
               app::ScenarioService& scenarios,
               app::ExperimentRunner& runner,
               infra::QtRenderer& renderer,
               QWidget* parent = nullptr);

    /// 算法进程是否在线（HUD 与状态栏共用）。只给布尔量：文案由界面层自己拼，
    /// 以前这里给的是可读字符串，结果 HUD 和状态栏各写了一套话术，同一种状态
    /// 能显示成两种说法。
    void setAlgoOnlineProbe(std::function<bool()> probe);

    /// 网关有没有监听上。没监听上时算法进程根本连不进来（多半是端口被占），
    /// 这与"在监听但算法还没起"是两回事：HUD 上必须分开说，否则用户会去查算法。
    /// 监听结果在启动时定一次，运行中不会变，所以直接传值而不是给探针。
    void setGatewayInfo(bool listening, const QString& note);

    /// 启动时载入的场景文件；为空则使用内置的跟车场景。
    void loadScenarioFile(const QString& path);

    /// 跳到场景选择页（供命令行 --picker 与测试出图使用）。
    void showScenarioPicker();

    /// 开始仿真（供命令行 --auto-scene 复现"点场景后自动开跑"）。
    void startSimulation();

signals:
    /// 第一帧真正贴到控件上时发一次。
    /// 冷启动时间必须量到这一帧：窗口 show() 之后到首帧之间还有场景加载、
    /// 首帧光栅化与一次跨线程回传，只记"进程起来了"会把这几百毫秒漏掉。
    void firstFrameReady();

private:
    void buildUi();
    void wireSignals();
    bool applyLoadedScenario(const QString& path);
    void refreshStatus();
    /// 把滑块范围与取值同步到回放窗口（earliest..latest）。
    void syncTimeline();

    void onTick();
    void onPlayPause();
    void onStep();
    void onSeek(int frame);
    void onJumpLatest();
    void onSpeedChanged(int index);
    void onOpenScenario();
    void onSandbox();
    void onClearGhosts();
    void onInjectAt(const QPoint& pos);
    void onRunExperiment();

    /// 首页入口：进入场景选择页 / 进入沙盒。
    void onScenarioMode();
    void onScenarioChosen(const QString& id);
    void onBackHome();
    /// 切到仿真视图页。
    void showSimView();

    /// 把视口中心移到自车位置。跟随关闭时不做任何事。
    void centerOnEgo();

    /// 从场景仓储收集预设场景，填进场景选择页。
    void populatePicker();

    app::SimController& sim_;
    app::ScenarioService& scenarios_;
    app::ExperimentRunner& runner_;
    infra::QtRenderer& renderer_;

    /// 三页：0 首页，1 场景选择，2 仿真视图。
    QStackedWidget* stack_{nullptr};
    HomeWidget* home_{nullptr};
    ScenarioPickerWidget* picker_{nullptr};

    ViewportWidget* view_{nullptr};
    QToolBar* toolbar_{nullptr};
    QSlider* timeline_{nullptr};
    QComboBox* speedBox_{nullptr};
    QLabel* frameLabel_{nullptr};
    QLabel* timeLabel_{nullptr};
    QLabel* fpsLabel_{nullptr};
    QLabel* cloudLabel_{nullptr};
    QLabel* linkLabel_{nullptr};
    QTimer* tick_{nullptr};

    std::function<bool()> algoOnlineProbe_;
    bool gatewayListening_{true};
    QString gatewayNote_;
    /// 本次运行里是否成功连上过算法；HUD 用它区分"还没接入"和"接入后掉了"。
    bool algoEverConnected_{false};
    QString scenarioPath_;

    QAction* followAction_{nullptr};

    bool userSeeking_{false};
    bool firstFrameSeen_{false};
    double renderMs_{0.0};
    int pointCount_{0};
    int framesThisSecond_{0};
    double fpsWindowStart_{0.0};
    double fps_{0.0};
};

}  // namespace adasim::ui
