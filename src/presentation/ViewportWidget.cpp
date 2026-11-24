#include "presentation/ViewportWidget.h"

#include <QFont>
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

void ViewportWidget::setHud(const HudState& hud) {
    hud_ = hud;
    update();
}

QPointF ViewportWidget::pixelToWorld(const QPoint& px) const {
    const double mpp = viewport_.metersPerPixel > 1e-9 ? viewport_.metersPerPixel : 0.1;
    return QPointF(viewport_.centerX + (px.x() - width() * 0.5) * mpp,
                   viewport_.centerY - (px.y() - height() * 0.5) * mpp);
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

    paintHud(painter);
}

void ViewportWidget::paintHud(QPainter& painter) {
    // HUD 画在控件坐标系里（不是图像坐标系）：图像随窗口缩放，
    // 而读数必须保持固定字号，否则窗口一大字就糊。
    painter.setRenderHint(QPainter::Antialiasing, true);

    const int pad = 10;
    const int w = 214;
    // 高度按行数算：车速块 30 + 6 行 × 17 + 上下留白。算法状态与 a/δ 拆成两行后
    // 比原来多一行，写死旧值会把"注入/点云"那行压到面板外。
    const int h = 153;
    const QRectF panel(pad, pad, w, h);

    painter.setPen(QPen(QColor(48, 54, 61), 1.0));
    painter.setBrush(QColor(13, 17, 23, 208));
    painter.drawRoundedRect(panel, 8.0, 8.0);

    QFont small = painter.font();
    small.setPixelSize(11);
    QFont big = painter.font();
    big.setPixelSize(25);
    big.setBold(true);

    const QColor dim(139, 148, 158);
    const QColor text(230, 237, 243);
    const QColor accent(88, 166, 255);
    const QColor warn(240, 136, 62);

    int y = pad + 14;

    // 车速：HUD 的主读数，给到 km/h——m/s 是仿真单位，km/h 才是驾驶员的直觉
    painter.setFont(big);
    painter.setPen(text);
    painter.drawText(QPointF(pad + 12, y + 16), QString::number(hud_.speedMps * 3.6, 'f', 0));
    painter.setFont(small);
    painter.setPen(dim);
    painter.drawText(QPointF(pad + 66, y + 16), QStringLiteral("km/h"));
    painter.drawText(QPointF(pad + 116, y + 16),
                     QStringLiteral("%1 m/s").arg(hud_.speedMps, 0, 'f', 1));

    // 暂停/回放状态：拖进度条回看时，画面停在历史帧上，
    // 不标出来会被误读成"仿真卡死了"。
    if (hud_.paused) {
        painter.setPen(warn);
        painter.drawText(QPointF(pad + w - 46, y + 16), QStringLiteral("暂停"));
    }

    y += 30;
    painter.setPen(dim);
    painter.drawText(QPointF(pad + 12, y),
                     QStringLiteral("帧率 %1  光栅化 %2 ms")
                         .arg(hud_.fps, 0, 'f', 1)
                         .arg(hud_.renderMs, 0, 'f', 2));

    y += 17;
    if (hud_.rttMs >= 0.0) {
        painter.drawText(QPointF(pad + 12, y),
                         QStringLiteral("往返时延 %1 ms  指令龄 %2 帧")
                             .arg(hud_.rttMs, 0, 'f', 2)
                             .arg(hud_.cmdAgeFrames));
    } else {
        painter.drawText(QPointF(pad + 12, y), QStringLiteral("往返时延 —  指令龄 —"));
    }

    y += 17;
    // 四态：网关没监听 / 在监听但没人连 / 曾连上后失联 / 在线。
    // 第一种是端口或配置问题，第二种只是还没起算法进程，第三种是断链兜底——
    // 三种情况用户的下一步动作完全不同，不能压成一句"离线"。
    if (!hud_.gatewayListening) {
        const QColor danger(248, 81, 73);
        painter.setPen(danger);
        painter.drawText(QPointF(pad + 12, y),
                         hud_.gatewayNote.isEmpty() ? QStringLiteral("算法 ✕ 网关未监听")
                                                    : QStringLiteral("算法 ✕ %1").arg(hud_.gatewayNote));
        // 网关没监听时没有 a/δ 可言，留白比画一排 0.00 诚实。
        y += 17;
        painter.setPen(dim);
        painter.drawText(QPointF(pad + 12, y),
                         QStringLiteral("注入 %1 个   点云 %2")
                             .arg(hud_.ghostCount)
                             .arg(hud_.pointCount));
        return;
    }

    painter.setPen(hud_.algoOnline ? accent : warn);
    painter.drawText(
        QPointF(pad + 12, y),
        hud_.algoOnline          ? QStringLiteral("算法 ● 在线")
        : hud_.algoEverConnected ? QStringLiteral("算法 ○ 失联（兜底接管）")
                                 : QStringLiteral("算法 ○ 等待接入"));

    // 算法输出：加速度、转角、决策原因。这三个量就是"算法在想什么"的可见部分。
    // 单独一行：失联文案比"在线"长一倍，挤在同一行会和 a/δ 叠字。
    y += 17;
    painter.setPen(text);
    painter.drawText(QPointF(pad + 12, y),
                     QStringLiteral("a %1  δ %2")
                         .arg(hud_.accel >= 0.0 ? QStringLiteral("+") + QString::number(hud_.accel, 'f', 2)
                                                : QString::number(hud_.accel, 'f', 2))
                         .arg(hud_.steer, 0, 'f', 3));

    y += 17;
    painter.setPen(dim);
    const QString reason = hud_.reason.isEmpty() ? QStringLiteral("—") : hud_.reason;
    painter.drawText(QPointF(pad + 12, y),
                     QStringLiteral("决策 %1").arg(reason));

    y += 17;
    painter.drawText(QPointF(pad + 12, y),
                     QStringLiteral("注入 %1 个   点云 %2")
                         .arg(hud_.ghostCount)
                         .arg(hud_.pointCount));
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
    // 右键 = 在当前点位注入一个目标。不做右键菜单：注入本身要"点哪出哪"，
    // 弹一层菜单等于每次多一次点击，演示时节奏全断。清除走工具栏。
    if (event->button() == Qt::RightButton) {
        emit rightClicked(event->pos());
        event->accept();
        return;
    }
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
