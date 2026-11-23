#include "infrastructure/render/QtRenderer.h"

#include <QElapsedTimer>
#include <QPainter>
#include <QPolygonF>
#include <QRectF>
#include <algorithm>
#include <cmath>

namespace adasim::infra {
namespace {

// 强度分档着色：预先准备好四档颜色，避免逐点做浮点乘加。
// 用青蓝系而不是灰蓝，是为了和障碍物的填充色（72,104,152）拉开对比——
// 早期版本两者色差过小，轮廓点压在车体上几乎看不出来。
constexpr QRgb kCloudPalette[4] = {
    qRgb(46, 116, 148),
    qRgb(78, 178, 208),
    qRgb(140, 224, 240),
    qRgb(230, 250, 255),
};

constexpr double kPi = 3.14159265358979323846;

constexpr QRgb kBackground = qRgb(18, 21, 26);
constexpr QRgb kGrid = qRgb(34, 39, 46);
constexpr QRgb kAxis = qRgb(52, 60, 70);
constexpr QRgb kEgoFill = qRgb(210, 72, 72);
constexpr QRgb kEgoEdge = qRgb(255, 168, 168);
constexpr QRgb kObstacleFill = qRgb(72, 104, 152);
constexpr QRgb kObstacleEdge = qRgb(140, 176, 220);

inline QRgb colorForIntensity(float k) {
    if (k < 0.35f) return kCloudPalette[0];
    if (k < 0.60f) return kCloudPalette[1];
    if (k < 0.82f) return kCloudPalette[2];
    return kCloudPalette[3];
}

}  // namespace

QtRenderer::QtRenderer(QObject* parent) : QObject(parent) {}

QtRenderer::~QtRenderer() = default;

void QtRenderer::submit(std::shared_ptr<const domain::WorldSnapshot> snapshot) {
    std::lock_guard<std::mutex> guard(mutex_);
    pending_ = std::move(snapshot);
}

void QtRenderer::setViewport(const domain::Viewport& viewport) {
    std::lock_guard<std::mutex> guard(mutex_);
    viewport_ = viewport;
}

domain::Viewport QtRenderer::viewport() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return viewport_;
}

QPointF QtRenderer::worldToPixel(double x, double y) const {
    const auto vp = viewport();
    const double mpp = vp.metersPerPixel > 1e-9 ? vp.metersPerPixel : 0.1;
    return QPointF((x - vp.centerX) / mpp + vp.widthPx * 0.5,
                   vp.heightPx * 0.5 - (y - vp.centerY) / mpp);
}

void QtRenderer::requestFrame() {
    std::shared_ptr<const domain::WorldSnapshot> snap;
    domain::Viewport vp;
    {
        std::lock_guard<std::mutex> guard(mutex_);
        snap = pending_;
        vp = viewport_;
    }
    if (!snap || vp.widthPx <= 0 || vp.heightPx <= 0) {
        return;
    }

    QElapsedTimer timer;
    timer.start();
    rasterize(*snap, vp);
    lastFrameMs_ = static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
    lastPointCount_ = static_cast<int>(snap->pointCount());

    emit frameReady(canvas_);
    emit frameStats(lastFrameMs_, lastPointCount_);
}

void QtRenderer::rasterize(const domain::WorldSnapshot& snap, const domain::Viewport& vp) {
    if (canvas_.width() != vp.widthPx || canvas_.height() != vp.heightPx) {
        canvas_ = QImage(vp.widthPx, vp.heightPx, QImage::Format_ARGB32_Premultiplied);
    }
    canvas_.fill(kBackground);

    const double mpp = vp.metersPerPixel > 1e-9 ? vp.metersPerPixel : 0.1;
    const double halfW = vp.widthPx * 0.5;
    const double halfH = vp.heightPx * 0.5;

    // 世界 → 像素的仿射变换，只算一次，后面全部复用。
    const auto toPx = [&](double wx, double wy, double& outX, double& outY) {
        outX = (wx - vp.centerX) / mpp + halfW;
        outY = halfH - (wy - vp.centerY) / mpp;
    };

    // ---- 1. 网格 ----
    {
        QPainter painter(&canvas_);
        painter.setRenderHint(QPainter::Antialiasing, false);

        // 网格间距随缩放自适应，避免缩小后密如噪点或放大后一片空白。
        double gridStep = 10.0;
        while (gridStep / mpp > 80.0) gridStep *= 0.5;
        while (gridStep / mpp < 24.0) gridStep *= 2.0;

        const double leftM = vp.centerX - halfW * mpp;
        const double rightM = vp.centerX + halfW * mpp;
        const double bottomM = vp.centerY - halfH * mpp;
        const double topM = vp.centerY + halfH * mpp;

        const double x0 = std::floor(leftM / gridStep) * gridStep;
        const double y0 = std::floor(bottomM / gridStep) * gridStep;

        for (double gx = x0; gx <= rightM; gx += gridStep) {
            double px = 0.0;
            double py = 0.0;
            toPx(gx, 0.0, px, py);
            const int col = static_cast<int>(std::lround(px));
            if (col < 0 || col >= vp.widthPx) continue;
            painter.setPen(QColor(std::abs(gx) < 1e-6 ? kAxis : kGrid));
            painter.drawLine(col, 0, col, vp.heightPx);
        }
        for (double gy = y0; gy <= topM; gy += gridStep) {
            double px = 0.0;
            double py = 0.0;
            toPx(0.0, gy, px, py);
            const int row = static_cast<int>(std::lround(py));
            if (row < 0 || row >= vp.heightPx) continue;
            painter.setPen(QColor(std::abs(gy) < 1e-6 ? kAxis : kGrid));
            painter.drawLine(0, row, vp.widthPx, row);
        }
    }

    // ---- 2. 障碍物 ----
    {
        QPainter painter(&canvas_);
        painter.setRenderHint(QPainter::Antialiasing, true);

        for (const auto& e : snap.state.obstacles) {
            double px = 0.0;
            double py = 0.0;
            toPx(e.pose.x, e.pose.y, px, py);

            // 屏幕 Y 轴向下，而航向角以世界坐标逆时针为正，故取负。
            painter.save();
            painter.translate(px, py);
            painter.rotate(-e.pose.theta * 180.0 / kPi);

            const double wPx = e.length / mpp;
            const double hPx = e.width / mpp;
            const QRectF box(-wPx * 0.5, -hPx * 0.5, wPx, hPx);
            painter.setBrush(QColor(kObstacleFill));
            painter.setPen(QPen(QColor(kObstacleEdge), 1.0));
            painter.drawRoundedRect(box, 2.0, 2.0);

            // 车头朝向：一条指向 +x 的短线，便于在缩小时仍能看出方向。
            painter.setPen(QPen(QColor(kObstacleEdge), 1.5));
            painter.drawLine(QPointF(wPx * 0.5, 0.0), QPointF(wPx * 0.35, -hPx * 0.3));
            painter.restore();
        }
    }

    // ---- 3. 点云：直接写扫描线 ----
    // 顺序上必须排车体之后：点云采的是障碍物轮廓，若先画点再画车，
    // 填充的车体会把轮廓点整片盖掉，画面上看起来"点云根本没显示"。
    // 这也是点云查看器的通行做法——观测值压在实体之上。
    //
    // 性能上这是关键路径。逐点 drawPoint 在十万点下约 60~90 ms，
    // 而直接写像素在同一台机器上约 2~4 ms。
    //
    // 每个点写成 2×2 像素块而不是单像素：单像素点在 0.1 m/px 缩放下
    // 只占一个点，被抗锯齿车身描边一压就看不出来，视觉上"像没画"。
    // 四倍写量在十万点下仍只增加约 1 ms，换来的是点云清晰可辨。
    const std::size_t n = snap.pointCount();
    if (n > 0) {
        const float* px = snap.px.data();
        const float* py = snap.py.data();
        const float* pi = snap.intensity.data();

        const double invMpp = 1.0 / mpp;
        // 留出一列/一行余量，保证 row+1、col+1 不越界，省掉逐次判断
        const int wLimit = vp.widthPx - 2;
        const int hLimit = vp.heightPx - 2;

        for (std::size_t i = 0; i < n; ++i) {
            const int col = static_cast<int>((px[i] - vp.centerX) * invMpp + halfW);
            if (col < 0 || col > wLimit) continue;
            const int row = static_cast<int>(halfH - (py[i] - vp.centerY) * invMpp);
            if (row < 0 || row > hLimit) continue;

            const QRgb c = colorForIntensity(pi[i]);
            auto* line0 = reinterpret_cast<QRgb*>(canvas_.scanLine(row));
            auto* line1 = reinterpret_cast<QRgb*>(canvas_.scanLine(row + 1));
            line0[col] = c;
            line0[col + 1] = c;
            line1[col] = c;
            line1[col + 1] = c;
        }
    }

    // ---- 4. 自车：最后画，保证压在点云与障碍物之上 ----
    {
        QPainter painter(&canvas_);
        painter.setRenderHint(QPainter::Antialiasing, true);

        double ex = 0.0;
        double ey = 0.0;
        toPx(snap.state.ego.x, snap.state.ego.y, ex, ey);

        painter.save();
        painter.translate(ex, ey);
        painter.rotate(-snap.state.ego.theta * 180.0 / kPi);
        const double egoW = 4.5 / mpp;
        const double egoH = 1.9 / mpp;
        painter.setBrush(QColor(kEgoFill));
        painter.setPen(QPen(QColor(kEgoEdge), 1.5));
        painter.drawRoundedRect(QRectF(-egoW * 0.5, -egoH * 0.5, egoW, egoH), 2.0, 2.0);
        painter.drawLine(QPointF(egoW * 0.5, 0.0), QPointF(egoW * 0.2, -egoH * 0.35));
        painter.drawLine(QPointF(egoW * 0.5, 0.0), QPointF(egoW * 0.2, egoH * 0.35));
        painter.restore();
    }
}

}  // namespace adasim::infra
