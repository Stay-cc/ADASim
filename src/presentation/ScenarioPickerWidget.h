#pragma once

#include <QWidget>
#include <vector>

#include "domain/scenario/ScenarioTemplate.h"

class QGridLayout;

namespace adasim::ui {

/// 场景选择页：网格列出所有预设场景，每张卡片带一张俯视示意图与名称，
/// 点击卡片即请求加载并开始仿真。
///
/// 卡片列表由外部传入（MainWindow 从场景仓储取），本类只负责展示与发信号，
/// 不认识持久化实现。缩略图是 QPainter 简笔俯视图：按道路曲率与参与者
/// 位姿画道路轮廓和几辆车，足够让用户在列表里一眼认出场景，而不是
/// 用"场景名"去猜内容。
class ScenarioPickerWidget final : public QWidget {
    Q_OBJECT

public:
    /// 场景条目：id（文件名）、显示名、以及用于画缩略图的模板。
    struct Entry {
        std::string id;
        QString name;
        QString description;
        domain::ScenarioTemplate tmpl;
    };

    explicit ScenarioPickerWidget(QWidget* parent = nullptr);

    /// 设置场景列表并重建网格。
    void setEntries(std::vector<Entry> entries);

signals:
    /// 用户点击了某张卡片，参数为场景文件 id（如 "following_lead"）。
    void scenarioChosen(const QString& id);
    /// 用户点「返回」，回到首页。
    void backRequested();

private:
    QWidget* buildCard(const Entry& entry);

    QWidget* grid_{nullptr};
    QGridLayout* gridLayout_{nullptr};
};

}  // namespace adasim::ui
