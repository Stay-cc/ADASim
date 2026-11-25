#include "presentation/MainWindow.h"

#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QStackedWidget>
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
#include "presentation/HomeWidget.h"
#include "presentation/ScenarioPickerWidget.h"
#include "presentation/ViewportWidget.h"

namespace adasim::ui {

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

    // 首页与场景选择页用浅色卡片风格（跟随系统 light 主题），
    // 仿真视图保持深色画布（渲染器背景本来就是深色，是"路面"的视觉基调）。
    setStyleSheet(QStringLiteral(
        "#homeTitle { font-size: 44px; font-weight: 700; color: #1f2933; }"
        "#homeSubtitle { font-size: 16px; color: #52606d; }"
        "#homeHint { font-size: 13px; color: #7b8794; }"
        "#cardGlyph { font-size: 56px; }"
        "#cardTitle { font-size: 22px; font-weight: 600; color: #1f2933; }"
        "#cardDesc { font-size: 13px; color: #52606d; }"
        "QPushButton#scenarioCard, QPushButton#sandboxCard {"
        "  background: #ffffff; border: 1px solid #e4e7eb; border-radius: 12px; }"
        "QPushButton#scenarioCard:hover, QPushButton#sandboxCard:hover {"
        "  border: 1px solid #2563eb; background: #f8faff; }"
        "#pickerTitle { font-size: 22px; font-weight: 600; color: #1f2933; }"
        "QPushButton#scenarioCard { background: #ffffff; border: 1px solid #e4e7eb;"
        "  border-radius: 10px; }"
        "QPushButton#scenarioCard:hover { border: 1px solid #2563eb; }"
        "#cardName { font-size: 14px; font-weight: 600; color: #1f2933; }"
        "#cardDesc { font-size: 12px; color: #52606d; }"
        "QMainWindow { background: #f5f7fa; }"
        "QWidget { background: #f5f7fa; }"));

    // 启动先停在首页，让用户选择「模拟场景」或「自由沙盒」，
    // 而不是一进来就默认载入某个场景。默认场景仍保留在 onScenarioMode 之外的
    // 兜底路径（命令行直接指定场景文件时不经过首页）。
    stack_->setCurrentIndex(0);
    toolbar_->setVisible(false);
    statusBar()->setVisible(false);
    fpsWindowStart_ = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    tick_->start(16);
    refreshStatus();
}

void MainWindow::buildUi() {
    toolbar_ = addToolBar(QStringLiteral("控制"));
    toolbar_->setMovable(false);

    auto* playAction = toolbar_->addAction(QStringLiteral("播放"));
    playAction->setShortcut(Qt::Key_Space);
    connect(playAction, &QAction::triggered, this, &MainWindow::onPlayPause);

    auto* stepAction = toolbar_->addAction(QStringLiteral("单步"));
    stepAction->setShortcut(Qt::Key_Right);
    connect(stepAction, &QAction::triggered, this, &MainWindow::onStep);

    toolbar_->addSeparator();

    toolbar_->addWidget(new QLabel(QStringLiteral(" 倍速 "), this));
    speedBox_ = new QComboBox(this);
    speedBox_->addItem(QStringLiteral("0.5×"), 0.5);
    speedBox_->addItem(QStringLiteral("1×"), 1.0);
    speedBox_->addItem(QStringLiteral("2×"), 2.0);
    speedBox_->addItem(QStringLiteral("4×"), 4.0);
    speedBox_->setCurrentIndex(1);
    toolbar_->addWidget(speedBox_);
    connect(speedBox_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onSpeedChanged);

    toolbar_->addSeparator();

    auto* homeAction = toolbar_->addAction(QStringLiteral("返回首页"));
    homeAction->setToolTip(QStringLiteral("回到首页，重新选择模拟场景或自由沙盒"));
    connect(homeAction, &QAction::triggered, this, &MainWindow::onBackHome);

    auto* openAction = toolbar_->addAction(QStringLiteral("载入场景"));
    connect(openAction, &QAction::triggered, this, &MainWindow::onOpenScenario);

    auto* expAction = toolbar_->addAction(QStringLiteral("跑实验"));
    connect(expAction, &QAction::triggered, this, &MainWindow::onRunExperiment);

    toolbar_->addSeparator();

    auto* sandboxAction = toolbar_->addAction(QStringLiteral("沙盒"));
    sandboxAction->setToolTip(QStringLiteral("无尽沙盒：空路面，右键点哪就在哪注入一个目标"));
    connect(sandboxAction, &QAction::triggered, this, &MainWindow::onSandbox);

    auto* clearAction = toolbar_->addAction(QStringLiteral("清除注入"));
    clearAction->setToolTip(QStringLiteral("移除当前所有注入目标"));
    connect(clearAction, &QAction::triggered, this, &MainWindow::onClearGhosts);

    toolbar_->addSeparator();

    // 默认跟随自车：视口初始中心在原点，而自车 10 s 就能开出上百米，
    // 不跟随的话跑几秒后画面里只剩网格——这对演示与论文配图都是致命的。
    // 一旦用户拖拽或滚轮调整，就自动取消跟随，避免"手一松就被拉回去"。
    followAction_ = toolbar_->addAction(QStringLiteral("跟随自车"));
    followAction_->setCheckable(true);
    followAction_->setChecked(true);

    auto* central = new QWidget(this);
    auto* centralLay = new QVBoxLayout(central);
    centralLay->setContentsMargins(0, 0, 0, 0);
    centralLay->setSpacing(0);

    stack_ = new QStackedWidget(central);
    centralLay->addWidget(stack_, 1);

    // ---- 第 0 页：首页 ----
    home_ = new HomeWidget(stack_);
    stack_->addWidget(home_);

    // ---- 第 1 页：场景选择 ----
    picker_ = new ScenarioPickerWidget(stack_);
    stack_->addWidget(picker_);

    // ---- 第 2 页：仿真视图（视口 + 进度条） ----
    auto* simPage = new QWidget(stack_);
    auto* layout = new QVBoxLayout(simPage);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    view_ = new ViewportWidget(simPage);
    layout->addWidget(view_, 1);

    // 进度条按帧号取值：滑块值就是 FrameId，不必在"百分比 ↔ 帧"之间来回换算，
    // 也不会因为窗口滑动导致映射漂移。
    auto* timelineRow = new QWidget(simPage);
    auto* timelineLayout = new QHBoxLayout(timelineRow);
    timelineLayout->setContentsMargins(6, 0, 6, 4);
    timelineLayout->setSpacing(6);

    timeline_ = new QSlider(Qt::Horizontal, timelineRow);
    timeline_->setRange(0, 0);
    timeline_->setSingleStep(1);
    timeline_->setPageStep(25);
    timeline_->setToolTip(QStringLiteral("拖动回看历史；回看不销毁记录，按播放才从当前位置另起一支"));
    timelineLayout->addWidget(timeline_, 1);

    auto* latestButton = new QPushButton(QStringLiteral("回到最新"), timelineRow);
    connect(latestButton, &QPushButton::clicked, this, &MainWindow::onJumpLatest);
    timelineLayout->addWidget(latestButton);

    layout->addWidget(timelineRow);

    stack_->addWidget(simPage);

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

    connect(home_, &HomeWidget::scenarioModeRequested, this, &MainWindow::onScenarioMode);
    connect(home_, &HomeWidget::sandboxRequested, this, &MainWindow::onSandbox);
    connect(picker_, &ScenarioPickerWidget::scenarioChosen, this, &MainWindow::onScenarioChosen);
    connect(picker_, &ScenarioPickerWidget::backRequested, this, &MainWindow::onBackHome);

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

    connect(view_, &ViewportWidget::rightClicked, this, &MainWindow::onInjectAt);

    connect(&renderer_, &infra::QtRenderer::frameReady, this,
            [this](const QImage& image) {
                view_->setImage(image);
                if (!firstFrameSeen_) {
                    firstFrameSeen_ = true;
                    emit firstFrameReady();
                }
            });

    connect(&renderer_, &infra::QtRenderer::frameStats, this,
            [this](double millis, int points) {
                renderMs_ = millis;
                pointCount_ = points;
                ++framesThisSecond_;
            });

    // 拖拽即回看：按下暂停、边走边定位。走的是 previewSeek（无损），
    // 所以来回拖不会把历史抹掉；真正截断留到用户按播放的那一刻。
    connect(timeline_, &QSlider::sliderPressed, this, [this]() {
        userSeeking_ = true;
        sim_.pause();
    });
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

void MainWindow::setAlgoOnlineProbe(std::function<bool()> probe) {
    algoOnlineProbe_ = std::move(probe);
}

void MainWindow::setGatewayInfo(bool listening, const QString& note) {
    gatewayListening_ = listening;
    gatewayNote_ = note;
}

void MainWindow::loadScenarioFile(const QString& path) {
    scenarioPath_ = path;
    if (applyLoadedScenario(path)) {
        // 命令行/外部指定场景时不经过首页，直接进仿真视图。
        showSimView();
    }
}

bool MainWindow::applyLoadedScenario(const QString& path) {
    const auto tmpl = scenarios_.load(path.toStdString());
    if (!tmpl.has_value()) {
        QMessageBox::warning(this, QStringLiteral("载入失败"),
                             QStringLiteral("无法读取场景文件：\n%1").arg(path));
        return false;
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
        return false;
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
    syncTimeline();
    refreshStatus();
    return true;
}

void MainWindow::onTick() {
    const double now = QDateTime::currentMSecsSinceEpoch() / 1000.0;

    const bool advanced = sim_.stepIfDue(now);
    if (advanced) {
        renderer_.submit(sim_.snapshot());
        renderer_.requestFrame();
        centerOnEgo();

        if (!userSeeking_) {
            syncTimeline();
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

    // 状态栏与 HUD 用同一组事实拼文案：以前两处各写一套，HUD 说"失联"、
    // 状态栏还挂着"等待连接"，同一个状态两种说法，看着像两个 bug。
    QString link;
    if (!gatewayListening_) {
        link = QStringLiteral("通信未启动");
    } else if (algoOnlineProbe_ && algoOnlineProbe_()) {
        link = QStringLiteral("算法进程 ● 在线");
    } else if (algoEverConnected_) {
        link = QStringLiteral("算法进程 ○ 已断开");
    } else {
        link = QStringLiteral("算法进程 ○ 等待连接");
    }
    // 兜底制动必须显示出来：画面上的刹车若来自底座而不是算法进程，
    // 不标注就会被当成算法在起作用，答辩时说不清。
    if (sim_.standbyGuardActive()) {
        link += QStringLiteral(" · 兜底制动");
    }
    linkLabel_->setText(link);

    // HUD 每拍都刷新：它显示的是"现在这一帧"的读数，不是某个历史快照的。
    // 暂停时也要刷——否则停在历史帧上时，画面上还挂着最后一个实时读数。
    HudState hud;
    hud.paused = !sim_.running();
    hud.speedMps = sim_.current().egoSpeed;
    hud.simTime = sim_.current().simTime;
    hud.fps = fps_;
    hud.renderMs = renderMs_;
    hud.pointCount = pointCount_;
    hud.ghostCount = static_cast<int>(sim_.ghostCount());
    hud.algoOnline = algoOnlineProbe_ ? algoOnlineProbe_() : false;
    if (hud.algoOnline) {
        algoEverConnected_ = true;
    }
    hud.algoEverConnected = algoEverConnected_;
    hud.gatewayListening = gatewayListening_;
    hud.gatewayNote = gatewayNote_;

    const auto& cmd = sim_.lastCommand();
    if (cmd.seq > 0) {
        hud.rttMs = cmd.algoRttMs;
        hud.accel = cmd.targetAccel;
        hud.steer = cmd.steer;
        hud.reason = QString::fromStdString(cmd.reason);
        const domain::FrameId latest = sim_.bufferLatest();
        hud.cmdAgeFrames = cmd.echoFrameId <= latest
                               ? static_cast<int>(latest - cmd.echoFrameId)
                               : -1;
    }
    view_->setHud(hud);
}

void MainWindow::onPlayPause() {
    sim_.toggle();
    // 恢复时把节拍基准重置到当前时刻，否则暂停期间累积的时间会在
    // 一帧内被折算成几十步，表现为"一按播放就跳了一大段"。
    tick_->start(16);
}

void MainWindow::onStep() {
    sim_.pause();
    // 停在历史帧上单步：先截断，否则新帧会被追加到旧写指针处，
    // 帧号一下跳到时间轴末尾，看起来像"单步把车瞬移了"。
    sim_.truncateHere();
    sim_.stepOnce();
    renderer_.submit(sim_.snapshot());
    renderer_.requestFrame();

    syncTimeline();
    refreshStatus();
}

void MainWindow::onSeek(int frame) {
    if (frame < 0) {
        return;
    }
    // 用 previewSeek 而不是 seek：回看是无损的，来回拖不会把后面的记录吃掉。
    // 截断由 SimController::start() 在真正恢复播放时执行。
    sim_.pause();
    if (sim_.previewSeek(static_cast<domain::FrameId>(frame))) {
        renderer_.submit(sim_.snapshot());
        renderer_.requestFrame();
    }
    refreshStatus();
}

void MainWindow::syncTimeline() {
    // 下限跟着环形缓冲的窗口起点走：历史被覆盖后，滑块不应还能拖到已不存在的帧。
    const auto clampInt = [](domain::FrameId v) {
        return static_cast<int>(std::min<domain::FrameId>(v, static_cast<domain::FrameId>(INT_MAX)));
    };
    const int earliest = clampInt(sim_.windowStart());
    // 上限用"已录到的最后一帧"，不是当前帧：回看时当前帧在后面，
    // 拿它当上限会让滑块自己缩回去，再也拖不回刚才看的位置。
    const int latest = clampInt(sim_.bufferLatest());
    if (timeline_->minimum() != earliest || timeline_->maximum() != latest) {
        timeline_->setRange(earliest, latest);
    }
    timeline_->setValue(latest);
}

void MainWindow::onJumpLatest() {
    const domain::FrameId latest = sim_.bufferLatest();
    if (sim_.previewSeek(latest)) {
        renderer_.submit(sim_.snapshot());
        renderer_.requestFrame();
    }
    syncTimeline();
    refreshStatus();
}

void MainWindow::onInjectAt(const QPoint& pos) {
    if (view_ == nullptr) {
        return;
    }
    const QPointF world = view_->pixelToWorld(pos);

    // 修饰键选类型：默认轿车，Shift 卡车，Ctrl 行人。
    domain::EntityKind kind = domain::EntityKind::Car;
    const Qt::KeyboardModifiers mods = QApplication::keyboardModifiers();
    if (mods & Qt::ShiftModifier) {
        kind = domain::EntityKind::Truck;
    } else if (mods & Qt::ControlModifier) {
        kind = domain::EntityKind::Pedestrian;
    }

    // 朝向取自车航向：在弯道上右键时，注入的车应当顺着路走而不是横在路中间。
    sim_.injectGhost(world.x(), world.y(), sim_.current().ego.theta, kind);
    renderer_.submit(sim_.snapshot());
    renderer_.requestFrame();
    refreshStatus();
}

void MainWindow::onClearGhosts() {
    sim_.clearGhosts();
    renderer_.submit(sim_.snapshot());
    renderer_.requestFrame();
    refreshStatus();
}

void MainWindow::onScenarioMode() {
    populatePicker();
    stack_->setCurrentIndex(1);
    toolbar_->setVisible(false);
    statusBar()->setVisible(false);
}

void MainWindow::showScenarioPicker() {
    onScenarioMode();
}

void MainWindow::populatePicker() {
    std::vector<ScenarioPickerWidget::Entry> entries;
    for (auto& s : scenarios_.listAvailable()) {
        // 沙盒单独走首页的「自由沙盒」入口，不混进预设场景列表。
        if (s.id == "sandbox") {
            continue;
        }
        ScenarioPickerWidget::Entry e;
        e.id = std::move(s.id);
        e.name = QString::fromStdString(s.name);
        e.description = QString::fromStdString(s.description);
        e.tmpl = std::move(s.tmpl);
        entries.push_back(std::move(e));
    }
    picker_->setEntries(std::move(entries));
}

void MainWindow::onScenarioChosen(const QString& id) {
    // 场景选择页给的是文件名（无扩展名，如 "following_lead"），补回 .json 再加载。
    const QString path = id.endsWith(QLatin1String(".json"))
                             ? id
                             : id + QLatin1String(".json");
    if (!applyLoadedScenario(path)) {
        return;
    }
    sim_.start();   // 从选择页进入即开始仿真
    showSimView();
}

void MainWindow::onBackHome() {
    // 回首页前暂停仿真：否则场景仍在后台推进，且首页上看不到任何反馈，
    // 再次进入时车已经跑出去很远，像个"没停过"的幽灵。
    sim_.pause();
    stack_->setCurrentIndex(0);
    toolbar_->setVisible(false);
    statusBar()->setVisible(false);
}

void MainWindow::showSimView() {
    stack_->setCurrentIndex(2);
    toolbar_->setVisible(true);
    statusBar()->setVisible(true);
}

void MainWindow::onSandbox() {
    // 沙盒走场景文件而不是内置模板：场地参数（车道数、雷达量程、分段道路）
    // 应当和别的场景一样可改可版本化，写死在代码里就只能重编译。
    // 路径要上三级（bin → build → 项目根），写成 ../scenarios 会指向
    // build/*/scenarios（不存在），沙盒就加载失败进不去。
    const QString exeDir = QCoreApplication::applicationDirPath();
    QString path = QDir(exeDir).absoluteFilePath(QStringLiteral("../../../scenarios/sandbox.json"));
    if (!QFileInfo::exists(path)) {
        path = QDir(exeDir).absoluteFilePath(QStringLiteral("../scenarios/sandbox.json"));
    }
    if (!QFileInfo::exists(path)) {
        path = QDir::current().absoluteFilePath(QStringLiteral("scenarios/sandbox.json"));
    }
    if (applyLoadedScenario(path)) {
        sim_.start();   // 沙盒载入即跑，不必再按一次播放
        showSimView();
    }
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
