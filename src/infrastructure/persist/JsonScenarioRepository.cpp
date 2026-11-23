#include "infrastructure/persist/JsonScenarioRepository.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QStringList>
#include <utility>

namespace adasim::infra {
namespace {

QString behaviorToString(domain::BehaviorKind kind) {
    switch (kind) {
        case domain::BehaviorKind::Constant: return QStringLiteral("constant");
        case domain::BehaviorKind::CarFollowing: return QStringLiteral("car_following");
        case domain::BehaviorKind::CutIn: return QStringLiteral("cut_in");
        case domain::BehaviorKind::Brake: return QStringLiteral("brake");
    }
    return QStringLiteral("constant");
}

domain::BehaviorKind behaviorFromString(const QString& s) {
    if (s == QLatin1String("car_following")) return domain::BehaviorKind::CarFollowing;
    if (s == QLatin1String("cut_in")) return domain::BehaviorKind::CutIn;
    if (s == QLatin1String("brake")) return domain::BehaviorKind::Brake;
    return domain::BehaviorKind::Constant;
}

// 读数值时给默认值而不是判错：场景文件通常是手写或由 AI 生成的，
// 缺一个可选字段就整个加载失败，用户体验很差。
double num(const QJsonObject& o, const char* key, double fallback) {
    const auto v = o.value(QLatin1String(key));
    return v.isDouble() ? v.toDouble() : fallback;
}

}  // namespace

JsonScenarioRepository::JsonScenarioRepository(std::string rootDir)
    : rootDir_(std::move(rootDir)) {}

QString JsonScenarioRepository::resolve(const std::string& path) const {
    const QString given = QString::fromStdString(path);
    const QFileInfo info(given);
    if (info.isAbsolute()) {
        return given;
    }
    return QDir(QString::fromStdString(rootDir_)).absoluteFilePath(given);
}

std::optional<domain::ScenarioTemplate> JsonScenarioRepository::load(const std::string& path) {
    const QString full = resolve(path);
    QFile file(full);
    if (!file.open(QIODevice::ReadOnly)) {
        lastError_ = "无法打开场景文件: " + full.toStdString();
        return std::nullopt;
    }

    QJsonParseError err{};
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) {
        lastError_ = "场景文件不是合法 JSON 对象: " + err.errorString().toStdString();
        return std::nullopt;
    }

    const QJsonObject root = doc.object();
    domain::ScenarioTemplate tmpl;
    tmpl.id = root.value(QStringLiteral("id")).toString().toStdString();
    if (tmpl.id.empty()) {
        tmpl.id = QFileInfo(full).completeBaseName().toStdString();
    }
    tmpl.name = root.value(QStringLiteral("name")).toString().toStdString();
    tmpl.laneCount = root.value(QStringLiteral("laneCount")).toInt(2);
    tmpl.laneWidth = num(root, "laneWidth", 3.5);
    tmpl.curvature = num(root, "curvature", 0.0);
    tmpl.friction = num(root, "friction", 0.9);
    tmpl.visibility = num(root, "visibility", 1.0);
    tmpl.durationSeconds = num(root, "durationSeconds", 30.0);
    tmpl.egoSpeedExplicit = root.value(QStringLiteral("egoSpeed")).isDouble();
    tmpl.egoSpeed = num(root, "egoSpeed", tmpl.egoSpeed);

    // 自车初始位姿可缺省。缺省为原点朝 +x，与早期场景文件语义一致。
    if (root.value(QStringLiteral("egoPose")).isObject()) {
        const QJsonObject e = root.value(QStringLiteral("egoPose")).toObject();
        tmpl.egoPose.x = num(e, "x", 0.0);
        tmpl.egoPose.y = num(e, "y", 0.0);
        tmpl.egoPose.theta = num(e, "theta", 0.0);
    }

    // sensor 块可缺省：缺省即传感器默认参数，老场景文件不必改。
    if (root.value(QStringLiteral("sensor")).isObject()) {
        const QJsonObject s = root.value(QStringLiteral("sensor")).toObject();
        tmpl.sensor.lidarRange = num(s, "lidarRange", tmpl.sensor.lidarRange);
        tmpl.sensor.lidarPointsPerMeter =
            num(s, "lidarPointsPerMeter", tmpl.sensor.lidarPointsPerMeter);
    }

    const QJsonArray actors = root.value(QStringLiteral("actors")).toArray();
    tmpl.actors.reserve(static_cast<std::size_t>(actors.size()));
    for (const auto& item : actors) {
        if (!item.isObject()) {
            continue;
        }
        const QJsonObject o = item.toObject();
        domain::ActorSpec spec;
        spec.type = o.value(QStringLiteral("type")).toString(QStringLiteral("car")).toStdString();
        spec.pose.x = num(o, "x", 0.0);
        spec.pose.y = num(o, "y", 0.0);
        spec.pose.theta = num(o, "theta", 0.0);
        spec.speed = num(o, "speed", 0.0);
        spec.behavior = behaviorFromString(o.value(QStringLiteral("behavior")).toString());
        spec.triggerSeconds = num(o, "trigger", 0.0);

        // 切入相关的可选字段。targetY 要区分"没写"与"写了 0"：
        // 没写时由车道宽推出目标车道，写了 0 则表示作者明确要求切到中心车道。
        spec.targetYExplicit = o.value(QStringLiteral("targetY")).isDouble();
        spec.targetY = num(o, "targetY", 0.0);
        spec.laneChangeSeconds = num(o, "laneChangeSeconds", 4.0);
        spec.brakeAccel = num(o, "brakeAccel", 0.0);
        spec.speedFloor = num(o, "speedFloor", 0.0);
        tmpl.actors.push_back(std::move(spec));
    }

    const QJsonArray params = root.value(QStringLiteral("parameters")).toArray();
    tmpl.parameters.reserve(static_cast<std::size_t>(params.size()));
    for (const auto& item : params) {
        if (!item.isObject()) {
            continue;
        }
        const QJsonObject o = item.toObject();
        domain::ParameterSpec spec;
        spec.name = o.value(QStringLiteral("name")).toString().toStdString();
        spec.min = num(o, "min", 0.0);
        spec.max = num(o, "max", 1.0);
        spec.step = num(o, "step", 0.1);
        if (spec.name.empty()) {
            continue;  // 无名参数无法被搜索器引用，直接跳过
        }
        tmpl.parameters.push_back(std::move(spec));
    }

    lastError_.clear();
    return tmpl;
}

bool JsonScenarioRepository::save(const std::string& path,
                                 const domain::ScenarioTemplate& tmpl) {
    const QString full = resolve(path);

    QJsonObject root;
    root.insert(QStringLiteral("id"), QString::fromStdString(tmpl.id));
    root.insert(QStringLiteral("name"), QString::fromStdString(tmpl.name));
    root.insert(QStringLiteral("laneCount"), tmpl.laneCount);
    root.insert(QStringLiteral("laneWidth"), tmpl.laneWidth);
    root.insert(QStringLiteral("curvature"), tmpl.curvature);
    QJsonObject egoPose;
    egoPose.insert(QStringLiteral("x"), tmpl.egoPose.x);
    egoPose.insert(QStringLiteral("y"), tmpl.egoPose.y);
    egoPose.insert(QStringLiteral("theta"), tmpl.egoPose.theta);
    root.insert(QStringLiteral("egoPose"), egoPose);
    root.insert(QStringLiteral("friction"), tmpl.friction);
    root.insert(QStringLiteral("visibility"), tmpl.visibility);
    root.insert(QStringLiteral("durationSeconds"), tmpl.durationSeconds);
    root.insert(QStringLiteral("egoSpeed"), tmpl.egoSpeed);

    QJsonObject sensor;
    sensor.insert(QStringLiteral("lidarRange"), tmpl.sensor.lidarRange);
    sensor.insert(QStringLiteral("lidarPointsPerMeter"), tmpl.sensor.lidarPointsPerMeter);
    root.insert(QStringLiteral("sensor"), sensor);

    QJsonArray actors;
    for (const auto& a : tmpl.actors) {
        QJsonObject o;
        o.insert(QStringLiteral("type"), QString::fromStdString(a.type));
        o.insert(QStringLiteral("x"), a.pose.x);
        o.insert(QStringLiteral("y"), a.pose.y);
        o.insert(QStringLiteral("theta"), a.pose.theta);
        o.insert(QStringLiteral("speed"), a.speed);
        o.insert(QStringLiteral("behavior"), behaviorToString(a.behavior));
        o.insert(QStringLiteral("trigger"), a.triggerSeconds);
        // 只在真正用到时写出，且 targetY 只在显式给出时写：
        // 无条件写 targetY=0 会让"继承车道默认值"的场景在存盘后变成"切到中心车道"。
        if (a.behavior == domain::BehaviorKind::CutIn) {
            if (a.targetYExplicit) {
                o.insert(QStringLiteral("targetY"), a.targetY);
            }
            o.insert(QStringLiteral("laneChangeSeconds"), a.laneChangeSeconds);
            o.insert(QStringLiteral("brakeAccel"), a.brakeAccel);
            o.insert(QStringLiteral("speedFloor"), a.speedFloor);
        }
        actors.append(o);
    }
    root.insert(QStringLiteral("actors"), actors);

    QJsonArray params;
    for (const auto& p : tmpl.parameters) {
        QJsonObject o;
        o.insert(QStringLiteral("name"), QString::fromStdString(p.name));
        o.insert(QStringLiteral("min"), p.min);
        o.insert(QStringLiteral("max"), p.max);
        o.insert(QStringLiteral("step"), p.step);
        params.append(o);
    }
    root.insert(QStringLiteral("parameters"), params);

    QDir().mkpath(QFileInfo(full).absolutePath());
    QFile file(full);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        lastError_ = "无法写入场景文件: " + full.toStdString();
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    file.close();

    lastError_.clear();
    return true;
}

std::vector<std::string> JsonScenarioRepository::listTemplates() const {
    std::vector<std::string> out;

    QDir dir(QString::fromStdString(rootDir_));
    if (!dir.exists()) {
        return out;
    }

    const QStringList filters{QStringLiteral("*.json")};
    const QFileInfoList files = dir.entryInfoList(filters, QDir::Files, QDir::Name);
    out.reserve(static_cast<std::size_t>(files.size()));
    for (const auto& info : files) {
        out.push_back(info.completeBaseName().toStdString());
    }
    return out;
}

}  // namespace adasim::infra
