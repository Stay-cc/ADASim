#pragma once

#include <optional>
#include <string>
#include <vector>

#include "domain/scenario/ScenarioTemplate.h"

namespace adasim::domain {

/// 持久化端口。场景模板与回放记录的读写都走这里，
/// 领域层不关心存储介质是 JSON、YAML 还是数据库。
struct IPersistence {
    virtual ~IPersistence() = default;

    [[nodiscard]] virtual std::optional<ScenarioTemplate> load(const std::string& path) = 0;
    virtual bool save(const std::string& path, const ScenarioTemplate& tmpl) = 0;

    /// 列出可用的模板标识，供场景编辑器展示。
    [[nodiscard]] virtual std::vector<std::string> listTemplates() const = 0;
};

}  // namespace adasim::domain
