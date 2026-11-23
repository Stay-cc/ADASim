#include "presentation/MainWindow.h"

#include <QAction>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QMessageBox>
#include <QSlider>
#include <QStatusBar>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWidget>
#include <algorithm>
#include <climits>

#include "application/ExperimentRunner.h"
#include "application/ScenarioService.h"
#include "application/SimController.h"
#include "infrastructure/render/QtRenderer.h"
#include "presentation/ViewportWidget.h"

namespace adasim::ui {
namespace {

/// 内置场景：本车 30 km/h 巡航，前方 25 m 一辆 15 km/h 的慢车。
/// 选它作为默认场景的原因是它会稳定触发减速与跟驰，
/// 启动后立刻能看到闭环在动，而不是一辆车对着空路直行。
domain::ScenarioTemplate makeDefaultScenario() {
    domain::ScenarioTemplate tmpl;
    tmpl.id = "default_following";
    tmpl.name = "默认跟车场景";
    tmpl.laneCount = 2;
    tmpl.laneWidth = 3.5;
    tmpl.durationSeconds = 60.0;

    domain::ActorSpec lead;
    lead.type = "car";
    lead.pose.x = 25.0;
    lead.pose.y = 0.0;
    lead.pose.theta = 0.0;
    lead.speed = 4.2;  // 约 15 km/h
    lead.behavior = domain::BehaviorKind::CarFollowing;
    tmpl.actors.push_back(lead);

    domain::ActorSpec side;
    side.type = "car";
    side.pose.x = 18.0;
    side.pose.y = 3.5;
    side.pose.theta = 0.0;
    side.speed = 8.0;
    side.behavior = domain::BehaviorKind::Constant;
    tmpl.actors.push_back(side);

    tmpl.parameters.push_back({"leadGap", 10.0, 40.0, 2.0});
    tmpl.parameters.push_back({"leadSpeed", 2.0, 12.0, 0.5});
    return tmpl;
}

}  // namespace

MainWindow::MainWindow(app::SimController& sim,
                       app::ScenarioService& scenarios,
                       app::ExperimentRunner& runner,
                       infra::QtRenderer& renderer,
                       QWidget* parent)
    : QMainWindow(parent),
      sim_(sim),
      scenarios_(scenarios),
      runner_(runner),
      renderer_(renderer) {
    buildUi();
    wireSignals();

    setWindowTitle(QStringLiteral("ADASim — 轻量级自动驾驶算法仿真平台"));
    resize(1180, 760);

    // 启动即载入默认场景，避免出现"空白窗口 + 需要先找文件"的冷启动体验。
    const auto tmpl = makeDefaultScenario();
    sim_.reset(scenarios_.buildInitialState(tmpl));
    sim_.pause();

    fpsWindowStart_ = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    tick_->start(16);
    refreshStatus();
}

void MainWindow::buildUi() {
    auto* toolbar = addToolBar(QStringLiteral("控制"));
    toolbar->setMovable(false);

    auto* playAction = toolbar->addAction(QStringLiteral("播放"));
    playAction->setShortcut(Qt::Key_Space);
    connect(playAction, &QAction::triggered, this, &MainWindow::onPlayPause);

    auto* stepAction = toolbar->addAction(QStringLiteral("单步"));
    stepAction->setShortcut(Qt::Key_Right);
    connect(stepAction, &QAction::triggered, this, &MainWindow::onStep);

    toolbar->addSeparator();

    toolbar->addWidget(new QLabel(QStringLiteral(" 倍速 "), this));
    speedBox_ = new QComboBox(this);
    speedBox_->addItem(QStringLiteral("0.5×"), 0.5);
    speedBox_->addItem(QStringLiteral("1×"), 1.0);
    speedBox_->addItem(QStringLiteral("2×"), 2.0);
    speedBox_->addItem(QStringLiteral("4×"), 4.0);
    speedBox_->setCurrentIndex(1);
    toolbar->addWidget(speedBox_);
    connect(speedBox_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onSpeedChanged);

    toolbar->addSeparator();

    auto* openAction = toolbar->addAction(QStringLiteral("载入场景"));
    connect(openAction, &QAction::triggered, this, &MainWindow::onOpenScenario);

    auto* expAction = toolbar->addAction(QStringLiteral("跑实验"));
    connect(expAction, &QAction::triggered, this, &MainWindow::onRunExperiment);

    toolbar->addSeparator();

    // 默认跟随自车：视口初始中心在原点，而自车 10 s 就能开出上百米，
    // 不跟随的话跑几秒后画面里只剩网格——这对演示与论文配图都是致命的。
    // 一旦用户拖拽或滚轮调整，就自动取消跟随，避免"手一松就被拉回去"。
    followAction_ = toolbar->addAction(QStringLiteral("跟随自车"));
    followAction_->setCheckable(true);
    followAction_->setChecked(true);

    auto* central = new QWidget(this);
    auto* layout = new QVBoxLayout(central);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    view_ = new ViewportWidget(central);
    layout->addWidget(view_, 1);

    timeline_ = new QSlider(Qt::Horizontal, central);
    timeline_->setRange(0, 0);
    timeline_->setSingleStep(1);
    timeline_->setPageStep(25);
    layout->addWidget(timeline_);

    setCentralWidget(central);

    frameLabel_ = new QLabel(this);
    timeLabel_ = new QLabel(this);
    fpsLabel_ = new QLabel(this);
    cloudLabel_ = new QLabel(this);
    linkLabel_ = new QLabel(this);
    statusBar()->addWidget(frameLabel_);
    statusBar()->addWidget(timeLabel_);
    statusBar()->addWidget(fpsLabel_);
    statusBar()->addWidget(cloudLabel_);
    statusBar()->addPermanentWidget(linkLabel_);

    tick_ = new QTimer(this);
}

void MainWindow::wireSignals() {
    connect(tick_, &QTimer::timeout, this, &MainWindow::onTick);

    connect(view_, &ViewportWidget::viewportChanged, this,
            [this](const domain::Viewport& vp) {
                renderer_.setViewport(vp);
                renderer_.requestFrame();
            });

    connect(view_, &ViewportWidget::viewAdjustedByUser, this, [this]() {
        if (followAction_ != nullptr) {
            followAction_->setChecked(false);
        }
    });

    connect(&renderer_, &infra::QtRenderer::frameReady, this,
            [this](const QImage& image) { view_->setImage(image); });

    connect(&renderer_, &infra::QtRenderer::frameStats, this,
            [this](double millis, int points) {
                renderMs_ = millis;
                pointCount_ = points;
                ++framesThisSecond_;
            });

    // 拖动时只暂停不回溯，松手后才 seek：否则每移动一像素就重推演一次，
    // 长窗口下会把界面拖到卡死。
    connect(timeline_, &QSlider::sliderPressed, this, [this]() { userSeeking_ = true; });
    connect(timeline_, &QSlider::sliderReleased, this, [this]() {
        userSeeking_ = false;
        onSeek(timeline_->value());
    });
    connect(timeline_, &QSlider::valueChanged, this, [this](int v) {
        if (userSeeking_) {
            onSeek(v);
        }
    });
}

void MainWindow::setLinkProbe(std::function<QString()> probe) {
    linkProbe_ = std::move(probe);
}

void MainWindow::loadScenarioFile(const QString& path) {
    scenarioPath_ = path;
    applyLoadedScenario(path);
}

void MainWindow::applyLoadedScenario(const QString& path) {
    const auto tmpl = scenarios_.load(path.toStdString());
    if (!tmpl.has_value()) {
        QMessageBox::warning(this, QStringLiteral("载入失败"),
                             QStringLiteral("无法读取场景文件：\n%1").arg(path));
        return;
    }

    // 语义校验放在这里而不是 JSON 层：车距为负、触发时刻超出时长这类问题
    // 需要领域知识，且要逐条提示给用户，不能只回一个"格式错误"。
    const auto problems = scenarios_.validate(*tmpl);
    if (!problems.empty()) {
        QString text = QStringLiteral("场景存在问题：\n");
        for (const auto& p : problems) {
            text += QStringLiteral("· ") + QString::fromStdString(p) + QLatin1Char('\n');
        }
        QMessageBox::warning(this, QStringLiteral("场景校验"), text);
        return;
    }

    domain::LidarParams lidar;
    lidar.range = tmpl->sensor.lidarRange;
    lidar.pointsPerMeter = tmpl->sensor.lidarPointsPerMeter;
    sim_.setLidarParams(lidar);

    sim_.reset(scenarios_.buildInitialState(*tmpl));
    sim_.pause();
    timeline_->setRange(0, 0);
    timeline_->setValue(0);
    renderer_.submit(sim_.snapshot());
    renderer_.requestFrame();
    refreshStatus();
}

void MainWindow::onTick() {
    const double now = QDateTime::currentMSecsSinceEpoch() / 1000.0;

    const bool advanced = sim_.stepIfDue(now);
    if (advanced) {
        renderer_.submit(sim_.snapshot());
        renderer_.requestFrame();
        centerOnEgo();

        if (!userSeeking_) {
            const int latest = static_cast<int>(std::min<domain::FrameId>(
                sim_.latestFrame(), static_cast<domain::FrameId>(INT_MAX)));
            if (timeline_->maximum() != latest) {
                timeline_->setRange(0, latest);
            }
            timeline_->setValue(latest);
        }
    }

    if (now - fpsWindowStart_ >= 1.0) {
        fps_ = framesThisSecond_ / (now - fpsWindowStart_);
        framesThisSecond_ = 0;
        fpsWindowStart_ = now;
    }
    refreshStatus();
}

void MainWindow::centerOnEgo() {
    if (followAction_ == nullptr || !followAction_->isChecked()) {
        return;
    }
    const domain::Pose& ego = sim_.current().ego;
    domain::Viewport vp = view_->viewport();
    if (vp.centerX == ego.x && vp.centerY == ego.y) {
        return;  // 停放时不必每帧重发视口变更
    }
    vp.centerX = ego.x;
    vp.centerY = ego.y;
    view_->setViewport(vp);   // 内部会触发 viewportChanged → 渲染端同步
}

void MainWindow::refreshStatus() {
    frameLabel_->setText(QStringLiteral("帧 %1").arg(static_cast<qulonglong>(sim_.latestFrame())));
    timeLabel_->setText(QStringLiteral("  仿真时间 %1 s")
                            .arg(sim_.current().simTime, 0, 'f', 2));
    fpsLabel_->setText(QStringLiteral("  %1 FPS / 光栅化 %2 ms")
                           .arg(fps_, 0, 'f', 1)
                           .arg(renderMs_, 0, 'f', 2));
    cloudLabel_->setText(QStringLiteral("  点云 %1 点").arg(pointCount_));

    if (linkProbe_) {
        QString link = linkProbe_();
        // 兜底制动必须显示出来：画面上的刹车若来自底座而不是算法进程，
        // 不标注就会被当成算法在起作用，答辩时说不清。
        if (sim_.standbyGuardActive()) {
            link += QStringLiteral(" · 兜底制动");
        }
        linkLabel_->setText(link);
    } else {
        linkLabel_->setText(QStringLiteral("未启用通信"));
    }
}

void MainWindow::onPlayPause() {
    sim_.toggle();
    // 恢复时把节拍基准重置到当前时刻，否则暂停期间累积的时间会在
    // 一帧内被折算成几十步，表现为"一按播放就跳了一大段"。
    tick_->start(16);
}

void MainWindow::onStep() {
    sim_.pause();
    sim_.stepOnce();
    renderer_.submit(sim_.snapshot());
    renderer_.requestFrame();

    const int latest = static_cast<int>(sim_.latestFrame());
    timeline_->setRange(0, latest);
    timeline_->setValue(latest);
    refreshStatus();
}

void MainWindow::onSeek(int frame) {
    if (frame < 0) {
        return;
    }
    sim_.pause();
    if (sim_.seek(static_cast<domain::FrameId>(frame))) {
        renderer_.submit(sim_.snapshot());
        renderer_.requestFrame();
    }
    refreshStatus();
}

void MainWindow::onSpeedChanged(int index) {
    const double factor = speedBox_->itemData(index).toDouble();
    sim_.setSpeedFactor(factor > 0.0 ? factor : 1.0);
}

void MainWindow::onOpenScenario() {
    const QString startDir = scenarioPath_.isEmpty()
                                 ? QDir::currentPath()
                                 : QFileInfo(scenarioPath_).absolutePath();
    const QString path = QFileDialog::getOpenFileName(
        this, QStringLiteral("选择场景文件"), startDir,
        QStringLiteral("场景 JSON (*.json);;所有文件 (*.*)"));
    if (path.isEmpty()) {
        return;
    }
    loadScenarioFile(path);
}

void MainWindow::onRunExperiment() {
    sim_.pause();

    const double duration = 30.0;
    const auto metrics = runner_.run(sim_.current(), duration, sim_.fixedStep());

    QString text;
    text += QStringLiteral("步数：%1\n").arg(static_cast<qulonglong>(metrics.steps));
    text += QStringLiteral("是否碰撞：%1\n").arg(metrics.collided ? QStringLiteral("是")
                                                                 : QStringLiteral("否"));
    text += QStringLiteral("最小 TTC：%1 s\n")
                .arg(metrics.minTtc > 1e8 ? 0.0 : metrics.minTtc, 0, 'f', 2);
    text += QStringLiteral("最大纵向加速度：%1 m/s²\n").arg(metrics.maxAbsAccel, 0, 'f', 2);
    text += QStringLiteral("行驶距离：%1 m\n").arg(metrics.travelled, 0, 'f', 1);
    text += QStringLiteral("真实耗时：%1 s\n").arg(metrics.wallSeconds, 0, 'f', 3);

    QMessageBox::information(this, QStringLiteral("离线实验结果（自当前状态起 30 s）"), text);
}

}  // namespace adasim::ui
