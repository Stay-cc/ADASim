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
constexpr QRgb kRoad = qRgb(32, 37, 45);
constexpr QRgb kLaneDash = qRgb(108, 120, 136);
constexpr QRgb kRoadEdge = qRgb(158, 170, 186);
constexpr QRgb kEgoFill = qRgb(210, 72, 72);
constexpr QRgb kEgoEdge = qRgb(255, 168, 168);
constexpr QRgb kObstacleFill = qRgb(72, 104, 152);
constexpr QRgb kObstacleEdge = qRgb(140, 176, 220);
constexpr QRgb kGhostEdge = qRgb(232, 162, 66);

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

    // ---- 1.5 路面与车道线 ----
    // 顺序：网格 → 路面 → 车道线 → 车体 → 点云。路面要压在网格上，
    // 否则网格线会横穿车道，看起来像车开在坐标纸上而不是路上。
    //
    // 车道纵向位置以"自车所在车道中心线"为基准：场景写的是 laneCount 与
    // laneWidth，没有横向偏移字段，约定自车沿本车道中心行驶即可。
    {
        QPainter painter(&canvas_);
        painter.setRenderHint(QPainter::Antialiasing, false);

        const auto& segs = snap.state.roadSegments;

        if (segs.empty()) {
            // 单曲率路径（旧场景）：整条路一条圆弧 / 直线，几何与算法侧 Stanley 同源。
            const int lanes = std::max(1, snap.state.laneCount);
            const double laneW = snap.state.laneWidth > 0.5 ? snap.state.laneWidth : 3.5;
            const double halfRoad = lanes * laneW * 0.5;
            const double kappa = snap.state.curvature;

            const auto boundaryY = [&](double wx, double d) {
                if (std::abs(kappa) < 1e-9) return d;
                const double radius = 1.0 / kappa;
                const double rr = radius + d;
                const double inner = rr * rr - wx * wx;
                return radius - (inner > 0.0 ? std::sqrt(inner) : 0.0);
            };

            const double leftM = vp.centerX - halfW * mpp;
            const double rightM = vp.centerX + halfW * mpp;
            QPolygonF road;
            const int samples = std::abs(kappa) < 1e-9 ? 1 : 24;
            road.reserve(samples * 2 + 2);
            for (int k = 0; k <= samples; ++k) {
                const double wx = leftM + (rightM - leftM) * k / samples;
                double px = 0.0, py = 0.0;
                toPx(wx, boundaryY(wx, halfRoad), px, py);
                road << QPointF(px, py);
            }
            for (int k = samples; k >= 0; --k) {
                const double wx = leftM + (rightM - leftM) * k / samples;
                double px = 0.0, py = 0.0;
                toPx(wx, boundaryY(wx, -halfRoad), px, py);
                road << QPointF(px, py);
            }
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(kRoad));
            painter.drawPolygon(road);

            painter.setPen(QPen(QColor(kLaneDash), 1.0, Qt::DashLine));
            for (int k = 1; k < lanes; ++k) {
                const double d = -halfRoad + k * laneW;
                QPolygonF line;
                for (int j = 0; j <= samples; ++j) {
                    const double wx = leftM + (rightM - leftM) * j / samples;
                    double px = 0.0, py = 0.0;
                    toPx(wx, boundaryY(wx, d), px, py);
                    line << QPointF(px, py);
                }
                painter.drawPolyline(line);
            }

            painter.setPen(QPen(QColor(kRoadEdge), 2.0));
            for (double d : {-halfRoad, halfRoad}) {
                QPolygonF line;
                for (int j = 0; j <= samples; ++j) {
                    const double wx = leftM + (rightM - leftM) * j / samples;
                    double px = 0.0, py = 0.0;
                    toPx(wx, boundaryY(wx, d), px, py);
                    line << QPointF(px, py);
                }
                painter.drawPolyline(line);
            }
        } else {
            // 分段道路（超大地图）：在里程空间采样中心线，逐点求世界位姿，
            // 垂直切线偏移出车道边界。相邻段曲率跳变时，边界线在交界处折一次，
            // 视觉上就是"S 弯"或"直道接弯道"。
            const double totalLen = domain::roadLength(segs);
            // 采样步长固定 2 m，既保证弯道平滑，又不至于点数爆炸。
            const double ds = 2.0;
            const int nSamples = std::max(2, static_cast<int>(totalLen / ds) + 1);

            // 先算出中心线所有采样点的世界坐标与切线角。
            std::vector<double> cx(nSamples), cy(nSamples), ct(nSamples);
            for (int i = 0; i < nSamples; ++i) {
                const double s = totalLen * i / (nSamples - 1);
                const domain::RoadPose rp = domain::roadPoseAt(segs, s);
                cx[i] = rp.x;
                cy[i] = rp.y;
                ct[i] = rp.theta;
            }

            // 车道数/宽度取自车当前路段（跟随自车所在段的几何）。
            const domain::RoadSegment* cur =
                domain::segmentAt(segs, snap.state.egoOdometer);
            const int lanes = std::max(1, cur ? cur->laneCount : snap.state.laneCount);
            const double laneW =
                cur && cur->laneWidth > 0.5 ? cur->laneWidth
                                            : (snap.state.laneWidth > 0.5 ? snap.state.laneWidth : 3.5);
            const double halfRoad = lanes * laneW * 0.5;

            // 法向偏移：世界坐标里，(cos(θ+90°), sin(θ+90°)) = (-sinθ, cosθ)。
            const auto offsetPt = [&](int i, double lateral, double& ox, double& oy) {
                ox = cx[i] - std::sin(ct[i]) * lateral;
                oy = cy[i] + std::cos(ct[i]) * lateral;
            };

            // 路面铺底：上边界 + 下边界围成多边形。
            QPolygonF road;
            road.reserve(nSamples * 2);
            for (int i = 0; i < nSamples; ++i) {
                double ox = 0.0, oy = 0.0, px = 0.0, py = 0.0;
                offsetPt(i, halfRoad, ox, oy);
                toPx(ox, oy, px, py);
                road << QPointF(px, py);
            }
            for (int i = nSamples - 1; i >= 0; --i) {
                double ox = 0.0, oy = 0.0, px = 0.0, py = 0.0;
                offsetPt(i, -halfRoad, ox, oy);
                toPx(ox, oy, px, py);
                road << QPointF(px, py);
            }
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(kRoad));
            painter.drawPolygon(road);

            // 车道分界虚线
            painter.setPen(QPen(QColor(kLaneDash), 1.0, Qt::DashLine));
            for (int k = 1; k < lanes; ++k) {
                const double lateral = -halfRoad + k * laneW;
                QPolygonF line;
                line.reserve(nSamples);
                for (int i = 0; i < nSamples; ++i) {
                    double ox = 0.0, oy = 0.0, px = 0.0, py = 0.0;
                    offsetPt(i, lateral, ox, oy);
                    toPx(ox, oy, px, py);
                    line << QPointF(px, py);
                }
                painter.drawPolyline(line);
            }

            // 路缘实线
            painter.setPen(QPen(QColor(kRoadEdge), 2.0));
            for (double lateral : {-halfRoad, halfRoad}) {
                QPolygonF line;
                line.reserve(nSamples);
                for (int i = 0; i < nSamples; ++i) {
                    double ox = 0.0, oy = 0.0, px = 0.0, py = 0.0;
                    offsetPt(i, lateral, ox, oy);
                    toPx(ox, oy, px, py);
                    line << QPointF(px, py);
                }
                painter.drawPolyline(line);
            }
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

    // ---- 2.5 注入目标（雷达欺骗）----
    // 用虚线 + 半透明填充，与实体车区分开：它不是刚体，物理世界并不存在，
    // 只有雷达能看见。画成实心的话，观众会以为"车真的撞上了"。
    if (!snap.state.ghosts.empty()) {
        QPainter painter(&canvas_);
        painter.setRenderHint(QPainter::Antialiasing, true);

        for (const auto& g : snap.state.ghosts) {
            double px = 0.0;
            double py = 0.0;
            toPx(g.pose.x, g.pose.y, px, py);

            painter.save();
            painter.translate(px, py);
            painter.rotate(-g.pose.theta * 180.0 / kPi);

            const double wPx = g.length / mpp;
            const double hPx = g.width / mpp;
            const QRectF box(-wPx * 0.5, -hPx * 0.5, wPx, hPx);
            painter.setBrush(QColor(232, 162, 66, 38));
            QPen pen(QColor(kGhostEdge), 1.4, Qt::DashLine);
            pen.setDashPattern({4.0, 3.0});
            painter.setPen(pen);
            painter.drawRect(box);

            // 打一个叉：一眼看出这是"标记"而不是一台车
            painter.drawLine(box.topLeft(), box.bottomRight());
            painter.drawLine(box.bottomLeft(), box.topRight());
            painter.restore();
        }
    }

    // ---- 3. 点云：直接写扫描线 ----
    // 顺序上必须排车体之后：点云采的是障碍物轮廓，若先画点再画车，
    // 填充的车体会把轮廓点整片盖掉，画面上看起来"点云根本没显示"。
    // 这也是点云查看器的通行做法——观测值压在实体之上。
    //
    // 性能上这是关键路径。逐点 drawPoint 在十万点下约 60~90 ms，
    // 而直接写像素实测 0.42 ms（2×2 块，十万点，见《性能压测报告》）——
    // 差两个数量级，根本原因是对每个点都调用了一次绘制状态机。
    //
    // 每个点写成 2×2 像素块而不是单像素：单像素点在 0.1 m/px 缩放下
    // 只占一个点，被抗锯齿车身描边一压就看不出来，视觉上"像没画"。
    // 四倍写量在十万点下仍只增加约 0.2 ms，换来的是点云清晰可辨。
    const std::size_t n = snap.pointCount();
    if (n > 0) {
        QElapsedTimer cloudTimer;
        cloudTimer.start();
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
        // 单独记点云段耗时：压测时要能分清"慢在点云"还是"慢在画车"，
        // 否则优化方向全是猜的。
        lastCloudMs_ = static_cast<double>(cloudTimer.nsecsElapsed()) / 1.0e6;
    } else {
        lastCloudMs_ = 0.0;
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
