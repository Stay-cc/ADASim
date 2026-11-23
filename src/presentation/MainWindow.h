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

    /// 由装配根注入链路状态探针，避免表现层认识通信实现。
    void setLinkProbe(std::function<QString()> probe);

    /// 启动时载入的场景文件；为空则使用内置的跟车场景。
    void loadScenarioFile(const QString& path);

private:
    void buildUi();
    void wireSignals();
    void applyLoadedScenario(const QString& path);
    void refreshStatus();

    void onTick();
    void onPlayPause();
    void onStep();
    void onSeek(int frame);
    void onSpeedChanged(int index);
    void onOpenScenario();
    void onRunExperiment();

    /// 把视口中心移到自车位置。跟随关闭时不做任何事。
    void centerOnEgo();

    app::SimController& sim_;
    app::ScenarioService& scenarios_;
    app::ExperimentRunner& runner_;
    infra::QtRenderer& renderer_;

    ViewportWidget* view_{nullptr};
    QSlider* timeline_{nullptr};
    QComboBox* speedBox_{nullptr};
    QLabel* frameLabel_{nullptr};
    QLabel* timeLabel_{nullptr};
    QLabel* fpsLabel_{nullptr};
    QLabel* cloudLabel_{nullptr};
    QLabel* linkLabel_{nullptr};
    QTimer* tick_{nullptr};

    std::function<QString()> linkProbe_;
    QString scenarioPath_;

    QAction* followAction_{nullptr};

    bool userSeeking_{false};
    double renderMs_{0.0};
    int pointCount_{0};
    int framesThisSecond_{0};
    double fpsWindowStart_{0.0};
    double fps_{0.0};
};

}  // namespace adasim::ui
