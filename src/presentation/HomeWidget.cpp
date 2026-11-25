#include "presentation/HomeWidget.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace adasim::ui {
namespace {

/// 一个大入口卡片按钮：图标文案 + 标题 + 说明。
QPushButton* makeCard(const QString& glyph, const QString& title,
                      const QString& desc, const QString& objectName,
                      QWidget* parent) {
    auto* btn = new QPushButton(parent);
    btn->setObjectName(objectName);
    btn->setCursor(Qt::PointingHandCursor);
    btn->setMinimumSize(300, 260);

    auto* lay = new QVBoxLayout(btn);
    lay->setContentsMargins(32, 40, 32, 32);
    lay->setSpacing(12);

    auto* glyphLabel = new QLabel(glyph, btn);
    glyphLabel->setAlignment(Qt::AlignCenter);
    glyphLabel->setObjectName(QStringLiteral("cardGlyph"));

    auto* titleLabel = new QLabel(title, btn);
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setObjectName(QStringLiteral("cardTitle"));

    auto* descLabel = new QLabel(desc, btn);
    descLabel->setAlignment(Qt::AlignCenter);
    descLabel->setWordWrap(true);
    descLabel->setObjectName(QStringLiteral("cardDesc"));

    lay->addWidget(glyphLabel);
    lay->addWidget(titleLabel);
    lay->addWidget(descLabel);
    lay->addStretch(1);
    return btn;
}

}  // namespace

HomeWidget::HomeWidget(QWidget* parent) : QWidget(parent) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(40, 32, 40, 32);
    root->setSpacing(8);

    auto* title = new QLabel(QStringLiteral("ADASim"), this);
    title->setObjectName(QStringLiteral("homeTitle"));
    title->setAlignment(Qt::AlignCenter);

    auto* subtitle = new QLabel(
        QStringLiteral("轻量级自动驾驶算法仿真平台"), this);
    subtitle->setObjectName(QStringLiteral("homeSubtitle"));
    subtitle->setAlignment(Qt::AlignCenter);

    root->addStretch(2);
    root->addWidget(title);
    root->addWidget(subtitle);
    root->addStretch(1);

    auto* cards = new QHBoxLayout();
    cards->setSpacing(24);
    cards->addStretch(1);

    auto* scenarioCard = makeCard(
        QStringLiteral("🗺"),
        QStringLiteral("模拟场景"),
        QStringLiteral("从场景库选择：跟驰、切入、紧急制动、\n弯道跟踪等预设工况，一键开始仿真。"),
        QStringLiteral("scenarioCard"), this);
    cards->addWidget(scenarioCard);

    auto* sandboxCard = makeCard(
        QStringLiteral("🧭"),
        QStringLiteral("自由沙盒"),
        QStringLiteral("超大地图：直道、弯道、多车道路段拼接，\n右键点击随时注入虚拟障碍物。"),
        QStringLiteral("sandboxCard"), this);
    cards->addWidget(sandboxCard);

    cards->addStretch(1);
    root->addLayout(cards);
    root->addStretch(2);

    auto* hint = new QLabel(
        QStringLiteral("模拟场景：预设工况 · 自由沙盒：运行时注入"), this);
    hint->setObjectName(QStringLiteral("homeHint"));
    hint->setAlignment(Qt::AlignCenter);
    root->addWidget(hint);
    root->addSpacing(8);

    connect(scenarioCard, &QPushButton::clicked, this,
            &HomeWidget::scenarioModeRequested);
    connect(sandboxCard, &QPushButton::clicked, this,
            &HomeWidget::sandboxRequested);
}

}  // namespace adasim::ui
