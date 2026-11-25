#include "presentation/ScenarioPickerWidget.h"

#include <QGridLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

#include "domain/model/RoadSegment.h"

namespace adasim::ui {
namespace {

constexpr double kPi = 3.14159265358979323846;

/// 场景缩略图：俯视简笔画。道路按曲率画一条中心线 + 两条边线，
/// 参与者画成小矩形（自车红色、其它蓝色），横向坐标按 y 缩放。
///
/// 世界坐标约定：自车沿 +x 前进，弯道是圆心在 (0, R=1/κ) 的圆弧。
/// 这里把世界 x∈[0, range] 映射到画布宽、y∈[-halfY, +halfY] 映射到画布高。
class Thumbnail : public QWidget {
public:
    explicit Thumbnail(const domain::ScenarioTemplate& tmpl, QWidget* parent = nullptr)
        : QWidget(parent), tmpl_(tmpl) {
        setFixedSize(220, 130);
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.fillRect(rect(), QColor(18, 21, 26));

        const double range = rangeM();
        const double halfY = halfYM();
        const int W = width();
        const int H = height();

        // 世界 → 画布：x 向右，y 向下。
        const auto toC = [&](double wx, double wy, double& cx, double& cy) {
            cx = (wx / range) * W;
            cy = H * 0.5 - (wy / halfY) * (H * 0.5);
        };

        // ---- 道路 ----
        p.setPen(Qt::NoPen);
        const auto laneCount = tmpl_.laneCount > 0 ? tmpl_.laneCount : 2;
        const double laneW = tmpl_.laneWidth > 0.5 ? tmpl_.laneWidth : 3.5;
        const double halfRoad = laneCount * laneW * 0.5;
        const double kappa = tmpl_.curvature;

        const auto boundaryY = [&](double wx, double d) {
            if (std::abs(kappa) < 1e-9) return d;
            const double R = 1.0 / kappa;
            const double rr = R + d;
            const double inner = rr * rr - wx * wx;
            return R - (inner > 0.0 ? std::sqrt(inner) : 0.0);
        };

        const int samples = 32;
        QPolygonF road;
        for (int k = 0; k <= samples; ++k) {
            const double wx = range * k / samples;
            double cx = 0.0, cy = 0.0;
            toC(wx, boundaryY(wx, halfRoad), cx, cy);
            road << QPointF(cx, cy);
        }
        for (int k = samples; k >= 0; --k) {
            const double wx = range * k / samples;
            double cx = 0.0, cy = 0.0;
            toC(wx, boundaryY(wx, -halfRoad), cx, cy);
            road << QPointF(cx, cy);
        }
        p.setBrush(QColor(40, 47, 58));
        p.drawPolygon(road);

        // 边线
        p.setPen(QPen(QColor(158, 170, 186), 1.5));
        for (double d : {-halfRoad, halfRoad}) {
            QPolygonF line;
            for (int k = 0; k <= samples; ++k) {
                const double wx = range * k / samples;
                double cx = 0.0, cy = 0.0;
                toC(wx, boundaryY(wx, d), cx, cy);
                line << QPointF(cx, cy);
            }
            p.drawPolyline(line);
        }

        // ---- 参与者 ----
        auto drawCar = [&](double x, double y, const QColor& fill, double len) {
            double cx = 0.0, cy = 0.0;
            toC(x, y, cx, cy);
            const double wPx = std::clamp(len / range * W, 4.0, 14.0);
            const double hPx = std::clamp(1.9 / halfY * H * 0.5, 2.5, 6.0);
            p.setBrush(fill);
            p.setPen(Qt::NoPen);
            p.drawRoundedRect(QRectF(cx - wPx * 0.5, cy - hPx * 0.5, wPx, hPx), 1.5, 1.5);
        };

        // 自车（红，原点附近）
        drawCar(4.0, tmpl_.egoPose.y, QColor(210, 72, 72), 4.5);
        for (const auto& a : tmpl_.actors) {
            drawCar(a.pose.x, a.pose.y, QColor(72, 104, 152),
                    a.type == "truck" ? 9.0 : 4.5);
        }
    }

private:
    double rangeM() const {
        // 覆盖到最远的参与者 + 自车前方一段距离。
        double far = 80.0;
        for (const auto& a : tmpl_.actors) {
            far = std::max(far, a.pose.x + 20.0);
        }
        return far;
    }
    double halfYM() const {
        const double laneCount = tmpl_.laneCount > 0 ? tmpl_.laneCount : 2;
        const double laneW = tmpl_.laneWidth > 0.5 ? tmpl_.laneWidth : 3.5;
        return laneCount * laneW;  // 上下各留出整条路宽的余量
    }

    domain::ScenarioTemplate tmpl_;
};

}  // namespace

ScenarioPickerWidget::ScenarioPickerWidget(QWidget* parent) : QWidget(parent) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(24, 16, 24, 16);
    root->setSpacing(12);

    auto* header = new QHBoxLayout();
    auto* title = new QLabel(QStringLiteral("选择模拟场景"), this);
    title->setObjectName(QStringLiteral("pickerTitle"));
    header->addWidget(title);
    header->addStretch(1);
    auto* back = new QPushButton(QStringLiteral("← 返回"), this);
    back->setCursor(Qt::PointingHandCursor);
    connect(back, &QPushButton::clicked, this, &ScenarioPickerWidget::backRequested);
    header->addWidget(back);
    root->addLayout(header);

    // 网格放进滚动区，场景多时不至于挤爆窗口。
    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setStyleSheet(QStringLiteral("QScrollArea{background:transparent;}"));

    auto* container = new QWidget(scroll);
    auto* containerLay = new QVBoxLayout(container);
    containerLay->setContentsMargins(0, 0, 0, 0);
    containerLay->addStretch(1);

    grid_ = new QWidget(container);
    gridLayout_ = new QGridLayout(grid_);
    gridLayout_->setSpacing(16);
    gridLayout_->setContentsMargins(0, 0, 0, 0);
    containerLay->insertWidget(0, grid_);

    scroll->setWidget(container);
    root->addWidget(scroll, 1);
}

QWidget* ScenarioPickerWidget::buildCard(const Entry& entry) {
    auto* btn = new QPushButton(this);
    btn->setCursor(Qt::PointingHandCursor);
    btn->setFixedSize(236, 210);
    btn->setObjectName(QStringLiteral("scenarioCard"));

    auto* lay = new QVBoxLayout(btn);
    lay->setContentsMargins(8, 8, 8, 10);
    lay->setSpacing(6);

    auto* thumb = new Thumbnail(entry.tmpl, btn);
    lay->addWidget(thumb, 0, Qt::AlignHCenter);

    auto* name = new QLabel(entry.name, btn);
    name->setObjectName(QStringLiteral("cardName"));
    name->setAlignment(Qt::AlignCenter);
    lay->addWidget(name);

    auto* desc = new QLabel(entry.description, btn);
    desc->setObjectName(QStringLiteral("cardDesc"));
    desc->setAlignment(Qt::AlignCenter);
    desc->setWordWrap(true);
    desc->setMaximumHeight(34);
    lay->addWidget(desc);

    const QString id = QString::fromStdString(entry.id);
    connect(btn, &QPushButton::clicked, this,
            [this, id]() { emit scenarioChosen(id); });
    return btn;
}

void ScenarioPickerWidget::setEntries(std::vector<Entry> entries) {
    // 清掉旧网格，重建。
    while (gridLayout_->count() > 0) {
        auto* item = gridLayout_->takeAt(0);
        if (item->widget()) {
            item->widget()->deleteLater();
        }
        delete item;
    }

    int col = 0;
    int row = 0;
    const int cols = 4;
    for (const auto& e : entries) {
        gridLayout_->addWidget(buildCard(e), row, col);
        if (++col >= cols) {
            col = 0;
            ++row;
        }
    }
}

}  // namespace adasim::ui
