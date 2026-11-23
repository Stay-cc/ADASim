#include "application/ScenarioService.h"

#include <cmath>
#include <set>

namespace adasim::app {
namespace {

constexpr double kDefaultEgoSpeed = 8.0;  // m/s，模板未给出时使用

domain::EntityKind kindFromString(const std::string& type) {
    if (type == "truck") {
        return domain::EntityKind::Truck;
    }
    if (type == "pedestrian") {
        return domain::EntityKind::Pedestrian;
    }
    if (type == "cyclist") {
        return domain::EntityKind::Cyclist;
    }
    if (type == "car") {
        return domain::EntityKind::Car;
    }
    return domain::EntityKind::Unknown;
}

}  // namespace

ScenarioService::ScenarioService(domain::IPersistence& store) : store_(store) {}

std::optional<domain::ScenarioTemplate> ScenarioService::load(const std::string& path) {
    auto tmpl = store_.load(path);
    if (!tmpl.has_value()) {
        return std::nullopt;
    }
    if (!validate(*tmpl).empty()) {
        return std::nullopt;  // 交由调用方通过 validate 获取具体原因
    }
    return tmpl;
}

std::vector<std::string> ScenarioService::validate(const domain::ScenarioTemplate& tmpl) const {
    std::vector<std::string> issues;

    if (tmpl.id.empty()) {
        issues.emplace_back("模板 id 不能为空");
    }
    if (tmpl.laneCount < 1) {
        issues.emplace_back("车道数必须大于等于 1");
    }
    if (tmpl.laneWidth <= 0.0) {
        issues.emplace_back("车道宽度必须为正");
    }
    if (!std::isfinite(tmpl.curvature)) {
        issues.emplace_back("曲率必须是有限值");
    }
    if (tmpl.friction <= 0.0 || tmpl.friction > 1.2) {
        issues.emplace_back("附着系数应在 (0, 1.2] 区间");
    }
    if (tmpl.visibility <= 0.0 || tmpl.visibility > 1.0) {
        issues.emplace_back("能见度系数应在 (0, 1] 区间");
    }
    if (tmpl.durationSeconds <= 0.0) {
        issues.emplace_back("场景时长必须为正");
    }
    if (tmpl.actors.empty()) {
        issues.emplace_back("场景至少需要一个交通参与者");
    }

    for (const auto& actor : tmpl.actors) {
        if (actor.speed < 0.0) {
            issues.emplace_back("参与者速度不能为负：" + actor.type);
        }
        if (actor.triggerSeconds < 0.0 || actor.triggerSeconds > tmpl.durationSeconds) {
            issues.emplace_back("触发时刻必须落在 [0, 场景时长] 内：" + actor.type);
        }
        const double reach = actor.pose.distanceTo(domain::Pose{});
        if (reach > 200.0) {
            issues.emplace_back("参与者初始位置距自车超过 200 m，可能不在量程内：" + actor.type);
        }
    }

    std::set<std::string> names;
    for (const auto& param : tmpl.parameters) {
        if (param.name.empty()) {
            issues.emplace_back("参数名不能为空");
            continue;
        }
        if (!names.insert(param.name).second) {
            issues.emplace_back("参数名重复：" + param.name);
        }
        if (param.min > param.max) {
            issues.emplace_back("参数下界大于上界：" + param.name);
        }
        if (param.step <= 0.0) {
            issues.emplace_back("参数步长必须为正：" + param.name);
        }
    }

    return issues;
}

bool ScenarioService::save(const std::string& path, const domain::ScenarioTemplate& tmpl) {
    if (!validate(tmpl).empty()) {
        return false;
    }
    return store_.save(path, tmpl);
}

domain::WorldState ScenarioService::buildInitialState(const domain::ScenarioTemplate& tmpl) const {
    domain::WorldState state;
    state.frameId = 0;
    state.simTime = 0.0;
    state.ego = domain::Pose{0.0, 0.0, 0.0};

    double egoSpeed = kDefaultEgoSpeed;
    for (const auto& param : tmpl.parameters) {
        if (param.name == "ego_speed") {
            egoSpeed = param.min;  // 默认取搜索区间下界，作为基线工况
            break;
        }
    }
    state.egoSpeed = egoSpeed;

    std::uint32_t id = 1;
    for (const auto& actor : tmpl.actors) {
        domain::EntityState entity;
        entity.id = id++;
        entity.kind = kindFromString(actor.type);
        entity.pose = actor.pose;
        entity.speed = actor.speed;
        entity.length = (entity.kind == domain::EntityKind::Truck) ? 9.0 : 4.5;
        entity.width = (entity.kind == domain::EntityKind::Truck) ? 2.5 : 1.9;
        state.obstacles.push_back(entity);
    }

    return state;
}

}  // namespace adasim::app
