#pragma once

#include <QImage>
#include <QObject>
#include <QPointF>
#include <memory>
#include <mutex>

#include "domain/port/IRenderPort.h"

namespace adasim::infra {

/// 离屏光栅化渲染器。
///
/// 刻意不继承 QWidget：本类只负责"把一帧画成一张 QImage"，
/// 贴到屏幕上是谁的事由表现层决定。这样做的原因是线程归属——
/// 光栅化可以在物理线程提交后立刻执行，而控件刷新必须在 GUI 线程，
/// 二者分离后就不存在"跨线程碰 QWidget"的问题。
///
/// 点云不走 QPainter::drawPoint：十万次 drawPoint 的调用开销足以吃掉
/// 整帧预算。这里改为直接写 QImage 的扫描线，按强度分档着色，
/// 一次遍历完成，实测可稳定在 8 ms 以内（见 6.1 实验）。
class QtRenderer final : public QObject, public domain::IRenderPort {
    Q_OBJECT

public:
    explicit QtRenderer(QObject* parent = nullptr);
    ~QtRenderer() override;

    void submit(std::shared_ptr<const domain::WorldSnapshot> snapshot) override;
    void setViewport(const domain::Viewport& viewport) override;
    void requestFrame() override;

    [[nodiscard]] domain::Viewport viewport() const;

    /// 最近一帧的光栅化耗时（毫秒），供性能面板与实验采集。
    [[nodiscard]] double lastFrameMillis() const noexcept { return lastFrameMs_; }

    /// 世界坐标 → 像素坐标，供界面做拾取（点选车辆、测距）。
    [[nodiscard]] QPointF worldToPixel(double x, double y) const;

signals:
    void frameReady(const QImage& image);
    void frameStats(double millis, int pointCount);

private:
    void rasterize(const domain::WorldSnapshot& snap, const domain::Viewport& vp);

    mutable std::mutex mutex_;
    std::shared_ptr<const domain::WorldSnapshot> pending_;
    domain::Viewport viewport_{};

    QImage canvas_;
    double lastFrameMs_{0.0};
    int lastPointCount_{0};
};

}  // namespace adasim::infra
