#pragma once

#include <QWidget>

namespace adasim::ui {

/// 首页面板：程序启动后的第一屏，给两个入口——
/// 「模拟场景」（进入场景选择页）与「自由沙盒」（直接进入超大地图）。
///
/// 它不认识仿真、不认识场景，只发两个信号；切页与加载都由 MainWindow 编排。
class HomeWidget final : public QWidget {
    Q_OBJECT

public:
    explicit HomeWidget(QWidget* parent = nullptr);

signals:
    /// 用户点了「模拟场景」。
    void scenarioModeRequested();
    /// 用户点了「自由沙盒」。
    void sandboxRequested();
};

}  // namespace adasim::ui
