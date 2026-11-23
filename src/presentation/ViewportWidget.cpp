#include "presentation/ViewportWidget.h"

#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

namespace adasim::ui {
namespace {
constexpr double kMinMetersPerPixel = 0.01;   // 放大极限：1 像素 1 cm
constexpr double kMaxMetersPerPixel = 4.0;    // 缩小极限：1 像素 4 m
}  // namespace

ViewportWidget::ViewportWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(480, 320);
    setFocusPolicy(Qt::StrongFocus);
    setAttribute(Qt::WA_OpaquePaintEvent, true);
}

void ViewportWidget::setImage(const QImage& image) {
    image_ = image;
    update();
}

void ViewportWidget::setViewport(const domain::Viewport& viewport) {
    viewport_ = viewport;
    emit viewportChanged(viewport_);
}

void ViewportWidget::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    viewport_.widthPx = width();
    viewport_.heightPx = height();
    emit viewportChanged(viewport_);
}

void ViewportWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    if (image_.isNull()) {
        painter.fillRect(rect(), QColor(18, 21, 26));
        painter.setPen(QColor(120, 130, 145));
        painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("等待场景加载"));
        return;
    }
    // 光栅化尺寸与控件尺寸一致时直接贴图，省一次缩放；
    // 尺寸不一致（例如拖动窗口的瞬间）才走平滑缩放。
    if (image_.size() == size()) {
        painter.drawImage(0, 0, image_);
    } else {
        painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
        painter.drawImage(rect(), image_);
    }
}

void ViewportWidget::wheelEvent(QWheelEvent* event) {
    const double steps = event->angleDelta().y() / 120.0;
    if (steps == 0.0) {
        return;
    }

    // 以光标位置为锚点缩放：先算出光标下的世界坐标，缩放后把它挪回原像素位置，
    // 否则放大时画面会往中心"跑"，用户需要反复平移来找回目标。
    const double before = viewport_.metersPerPixel;
    const double anchorPx = event->position().x();
    const double anchorPy = event->position().y();
    const double worldAnchorX = viewport_.centerX + (anchorPx - width() * 0.5) * before;
    const double worldAnchorY = viewport_.centerY - (anchorPy - height() * 0.5) * before;

    const double factor = std::pow(0.88, steps);
    viewport_.metersPerPixel =
        std::clamp(before * factor, kMinMetersPerPixel, kMaxMetersPerPixel);

    const double after = viewport_.metersPerPixel;
    viewport_.centerX = worldAnchorX - (anchorPx - width() * 0.5) * after;
    viewport_.centerY = worldAnchorY + (anchorPy - height() * 0.5) * after;

    emit viewportChanged(viewport_);
    emit viewAdjustedByUser();
    event->accept();
}

void ViewportWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        dragging_ = true;
        lastPos_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
    }
}

void ViewportWidget::mouseMoveEvent(QMouseEvent* event) {
    if (!dragging_) {
        return;
    }
    const QPoint delta = event->pos() - lastPos_;
    lastPos_ = event->pos();

    viewport_.centerX -= delta.x() * viewport_.metersPerPixel;
    viewport_.centerY += delta.y() * viewport_.metersPerPixel;
    emit viewportChanged(viewport_);
    emit viewAdjustedByUser();
}

void ViewportWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        dragging_ = false;
        unsetCursor();
    }
}

}  // namespace adasim::ui
