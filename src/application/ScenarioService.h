#pragma once

#include <optional>
#include <string>
#include <vector>

#include "domain/model/WorldState.h"
#include "domain/port/IPersistence.h"
#include "domain/scenario/ScenarioTemplate.h"

namespace adasim::app {

/// 场景服务：加载、校验、保存场景模板，并把模板实例化为初始世界状态。
///
/// 校验分成两层：结构校验（字段是否齐全、类型是否正确）由持久化实现负责，
/// 语义校验（车距是否为正、触发时刻是否落在时长内）在这里做——
/// 后者需要领域知识，不能下沉到 JSON 层。
class ScenarioService {
public:
    explicit ScenarioService(domain::IPersistence& store);

    [[nodiscard]] std::optional<domain::ScenarioTemplate> load(const std::string& path);

    /// 返回问题清单；空表示通过。不做异常抛出，便于界面逐条提示。
    [[nodiscard]] std::vector<std::string> validate(const domain::ScenarioTemplate& tmpl) const;

    bool save(const std::string& path, const domain::ScenarioTemplate& tmpl);

    /// 按模板构造初始帧。ego 置于原点、沿 +x 方向，障碍物按其位姿放置。
    [[nodiscard]] domain::WorldState buildInitialState(const domain::ScenarioTemplate& tmpl) const;

private:
    domain::IPersistence& store_;
};

}  // namespace adasim::app
