#pragma once

#include <QString>
#include <optional>
#include <string>
#include <vector>

#include "domain/port/IPersistence.h"

namespace adasim::infra {

/// JSON 场景仓库。
///
/// 只负责结构层：字段存在与否、类型对不对。语义层（车距为正、
/// 触发时刻落在时长内）留给应用层的 ScenarioService，
/// 因为那需要领域知识，放这里会让存储格式与业务规则绑死。
///
/// 一个模板一个文件，文件名即模板 id。这样做的好处是场景可以进版本库、
/// 可以人工 diff，也比塞进单个大 JSON 更容易合并冲突。
class JsonScenarioRepository final : public domain::IPersistence {
public:
    explicit JsonScenarioRepository(std::string rootDir);

    [[nodiscard]] std::optional<domain::ScenarioTemplate> load(const std::string& path) override;
    bool save(const std::string& path, const domain::ScenarioTemplate& tmpl) override;
    [[nodiscard]] std::vector<std::string> listTemplates() const override;

    [[nodiscard]] const std::string& rootDir() const noexcept { return rootDir_; }
    [[nodiscard]] const std::string& lastError() const noexcept { return lastError_; }

private:
    [[nodiscard]] QString resolve(const std::string& path) const;

    std::string rootDir_;
    mutable std::string lastError_;
};

}  // namespace adasim::infra
