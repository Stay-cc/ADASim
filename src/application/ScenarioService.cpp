#include "application/ScenarioService.h"

#include <cmath>
#include <set>

namespace adasim::app {
namespace {

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

/// 切入的默认目标横向坐标：朝自车所在车道（y=0）方向平移一个车道。
///
/// 场景作者通常只写"从哪条车道切进来"，目标车道是隐含的；
/// 要求他再写一个 y 值，等于强迫他把车道宽和车道索引算一遍，很容易写错。
double defaultCutInTargetY(double originY, double laneWidth) {
    if (std::abs(originY) < 1e-6) {
        return -laneWidth;   // 已经在中心车道：默认往左切一条
    }
    return originY - std::copysign(laneWidth, originY);
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
    // 自车初速既决定几何也决定时序：为负会让积分反向，过大则超出
    // 运动学模型的适用范围（本模型的轮胎侧偏假设到不了更高速度）。
    if (tmpl.egoSpeed < 0.0 || tmpl.egoSpeed > 60.0) {
        issues.emplace_back("自车初速应在 [0, 60] m/s");
    }
    // 传感器参数的上下界不是审美问题：密度过大会让一帧点云涨到百万级，
    // 直接把渲染与网络压垮；为负则采样循环会退化成空点云。
    if (tmpl.sensor.lidarRange <= 0.0 || tmpl.sensor.lidarRange > 500.0) {
        issues.emplace_back("激光雷达量程应在 (0, 500] 米");
    }
    if (tmpl.sensor.lidarPointsPerMeter <= 0.0 || tmpl.sensor.lidarPointsPerMeter > 100.0) {
        issues.emplace_back("点云采样密度应在 (0, 100] 点/米");
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

        // 切入必须真的"换道"：目标与起点重合时，那辆车会一直直行，
        // 场景看起来加载成功却什么也没发生——这类静默失败最难排查，故前置拦截。
        if (actor.behavior == domain::BehaviorKind::CutIn) {
            const double target =
                actor.targetYExplicit ? actor.targetY
                                      : defaultCutInTargetY(actor.pose.y, tmpl.laneWidth);
            if (std::abs(target - actor.pose.y) < 0.5) {
                issues.emplace_back("切入的起点与目标横向坐标过于接近：" + actor.type);
            }
            if (actor.laneChangeSeconds < 0.5 || actor.laneChangeSeconds > 30.0) {
                issues.emplace_back("切入耗时应在 [0.5, 30] 秒：" + actor.type);
            }
            if (actor.triggerSeconds >= tmpl.durationSeconds) {
                issues.emplace_back("切入触发时刻不早于场景结束，永远不会发生：" + actor.type);
            }
        }
        // 制动行为的两个参数缺一不可：没有 brakeAccel 就永远减不了速，
        // 表现为一辆匀速车——文件写得像"急刹"、跑起来却什么也没发生。
        if (actor.behavior == domain::BehaviorKind::Brake) {
            if (actor.brakeAccel <= 0.0) {
                issues.emplace_back("制动行为必须给出正的减速度 brakeAccel：" + actor.type);
            }
            if (actor.triggerSeconds >= tmpl.durationSeconds) {
                issues.emplace_back("制动触发时刻不早于场景结束，永远不会发生：" + actor.type);
            }
        }
        if (actor.brakeAccel < 0.0) {
            issues.emplace_back("减速度应填正值：" + actor.type);
        }
        if (actor.speedFloor < 0.0 || actor.speedFloor > actor.speed + 1e-9) {
            issues.emplace_back("制动目标车速不能为负，也不能高于初速：" + actor.type);
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
    state.ego = tmpl.egoPose;

    double egoSpeed = tmpl.egoSpeed;
    if (!tmpl.egoSpeedExplicit) {
        // 兼容旧场景：早期版本没有 egoSpeed 字段，自车初速只能从
        // parameters 里找一个叫 ego_speed 的项、取其搜索下界当基线。
        // 这条路径保留，但新场景应直接写 egoSpeed。
        for (const auto& param : tmpl.parameters) {
            if (param.name == "ego_speed") {
                egoSpeed = param.min;
                break;
            }
        }
    }
    state.egoSpeed = egoSpeed;
    // 附着系数随初始状态下发。此前它只存在于模板里、物理层根本没读，
    // 于是"路面湿滑"这个搜索变量无论怎么变，仿真结果都一模一样。
    state.friction = tmpl.friction;
    // 曲率同样随状态下发：算法侧要据此生成弯道参考路径，
    // 只存在模板里的话，仿真跑起来就只剩一条直道。
    state.curvature = tmpl.curvature;

    std::uint32_t id = 1;
    for (const auto& actor : tmpl.actors) {
        domain::EntityState entity;
        entity.id = id++;
        entity.kind = kindFromString(actor.type);
        entity.pose = actor.pose;
        entity.speed = actor.speed;
        entity.length = (entity.kind == domain::EntityKind::Truck) ? 9.0 : 4.5;
        entity.width = (entity.kind == domain::EntityKind::Truck) ? 2.5 : 1.9;

        // 行为按"实例"落到世界状态里：同一份模板跑两次，行为参数必须各自独立，
        // 否则回溯到中途再改参数会污染另一条时间线。
        domain::BehaviorState& b = entity.behavior;
        b.kind = actor.behavior;
        b.triggerSeconds = actor.triggerSeconds;
        b.originY = actor.pose.y;
        b.targetY = actor.targetYExplicit
                        ? actor.targetY
                        : defaultCutInTargetY(actor.pose.y, tmpl.laneWidth);
        b.laneChangeSeconds = actor.laneChangeSeconds;
        b.brakeAccel = actor.brakeAccel;
        b.speedFloor = actor.speedFloor;
        b.desiredSpeed = actor.speed;

        state.obstacles.push_back(entity);
    }

    return state;
}

}  // namespace adasim::app
