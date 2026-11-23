#pragma once

#include <QImage>
#include <QPoint>
#include <QWidget>

#include "domain/model/Viewport.h"

namespace adasim::ui {

/// 视口控件：只做三件事——贴图、缩放、平移。
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
    [[nodiscard]] const domain::Viewport& viewport() const noexcept { return viewport_; }

signals:
    void viewportChanged(const domain::Viewport& viewport);

    /// 仅由用户操作（滚轮/拖拽）触发，程序自身改视口时不发。
    /// 主窗口靠它关闭"跟随自车"——如果用 viewportChanged 判断，
    /// 初始化时的 resizeEvent 也会被误当成用户操作。
    void viewAdjustedByUser();

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    QImage image_;
    domain::Viewport viewport_{};
    bool dragging_{false};
    QPoint lastPos_{};
};

}  // namespace adasim::ui
