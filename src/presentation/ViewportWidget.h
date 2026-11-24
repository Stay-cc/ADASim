#pragma once

#include <QImage>
#include <QPoint>
#include <QPointF>
#include <QString>
#include <QWidget>

#include "domain/model/Viewport.h"

namespace adasim::ui {

/// HUD 上要显示的一组实时读数。
///
/// 刻意做成值类型而不是让控件去问 SimController：控件一旦持有仿真对象，
/// 它就再也离不开仿真了（例如做纯渲染截图时会牵出一串依赖）。
struct HudState {
    bool paused{false};
    double speedMps{0.0};
    double simTime{0.0};
    double fps{0.0};
    double renderMs{0.0};
    /// 算法侧实测的往返时延，ms；负值表示还没测到（未连接或刚连上）。
    double rttMs{-1.0};
    /// 指令相对当前帧的滞后帧数；-1 表示没有指令。
    int cmdAgeFrames{-1};
    bool algoOnline{false};
    /// 网关是否在监听。false 表示算法进程**根本连不上**（通常是端口被占用），
    /// 与「在监听但还没人连」必须分开显示：前者要用户去查端口，后者只要去起算法。
    /// 混成一句"离线"，两种情况的排查方向完全相反。
    bool gatewayListening{true};
    /// 未监听时的原因（含端口号），直接画在 HUD 上，省得去翻终端日志。
    QString gatewayNote;
    /// 本次运行里是否成功连上过算法。用它把"还没接入"和"接入后掉了"分开：
    /// 断链兜底是本平台的演示场景之一，掉线时显示"等待接入"会让人以为没连上过。
    bool algoEverConnected{false};
    double accel{0.0};
    double steer{0.0};
    QString reason;
    int ghostCount{0};
    int pointCount{0};
};

/// 视口控件：只做四件事——贴图、缩放、平移、画 HUD。
///
/// 真正的光栅化在 infrastructure::QtRenderer 里完成，本类不认识点云结构，
/// 也不碰物理状态。这条边界让渲染可以脱离界面单测：
/// 只要给 QtRenderer 一份快照，就能断言它在 8 ms 内出图。
class ViewportWidget final : public QWidget {
    Q_OBJECT

public:
    explicit ViewportWidget(QWidget* parent = nullptr);

    void setImage(const QImage& image);
    void setViewport(const domain::Viewport& viewport);
    void setHud(const HudState& hud);

    [[nodiscard]] const domain::Viewport& viewport() const noexcept { return viewport_; }

    /// 控件像素坐标 → 世界坐标。右键注入与后续的"点哪打哪"都靠它。
    [[nodiscard]] QPointF pixelToWorld(const QPoint& px) const;

signals:
    void viewportChanged(const domain::Viewport& viewport);

    /// 仅由用户操作（滚轮/拖拽）触发，程序自身改视口时不发。
    /// 主窗口靠它关闭"跟随自车"——如果用 viewportChanged 判断，
    /// 初始化时的 resizeEvent 也会被误当成用户操作。
    void viewAdjustedByUser();

    /// 右键单击：坐标为控件像素。主窗口负责换算成世界坐标再注入。
    void rightClicked(const QPoint& pos);

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    void paintHud(QPainter& painter);

    QImage image_;
    domain::Viewport viewport_{};
    HudState hud_{};
    bool dragging_{false};
    QPoint lastPos_{};
};

}  // namespace adasim::ui
