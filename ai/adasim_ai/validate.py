"""语义校验：结构合法之后，判断这个场景"物理上讲不讲得通"。

与结构校验的分工必须清楚，因为两者的处置方式完全不同：

  * 结构错（字段类型不对、必填缺失）→ 回填给语言模型重试，模型改得动；
  * 语义错（前车在自车后头、两车初始重叠）→ 模型往往改不动，因为这是
    它对物理世界的理解出了问题，重试十次还是错。这类应拦截并转人工。

因此 `generate` 只对结构错重试，语义错直接进入"标注待人工确认"流程。

本模块最值得说的一条是**轨迹交叉检测**。上一轮做 `cut_in_brake` 时踩过：
前车 30 m/5 m/s、切入车 12 m/9 m/s，两者在 t≈4 s 纵向重合而横向只差 2.1 m，
切入车直接从慢车身上穿过去。单测只检查"那辆车自己走得对不对"，查不出来；
加载也不报错；只在画面里看着别扭。而基于这种场景做失效搜索，结论全是假的。
所以这里按轨迹外推做包围盒互穿检查——代价是几分钟代码，收益是避免一整批
实验数据作废。
"""

from __future__ import annotations

import math
from typing import Any

from .schema import Issue, check_structure
from .template import Actor, Scenario

#: 自车包围盒（与 C++ `ScenarioService` 的默认车身一致）。
EGO_LENGTH = 4.5
EGO_WIDTH = 1.9

#: 轨迹外推的时间步长。0.1 s 足以捕捉相对速度 10 m/s 时的穿模
#: （每步位移 1 m，远小于车身长度）。
_PREDICT_DT = 0.1


def _actor_size(actor: Actor) -> tuple[float, float]:
    if actor.type == "truck":
        return 9.0, 2.5
    if actor.type in ("pedestrian", "cyclist"):
        return 0.8, 0.8
    return EGO_LENGTH, EGO_WIDTH


def _cut_in_ratio(progress: float) -> float:
    """切入横向进度。与 C++ `cutInLateralRatio` 同一条余弦曲线。

    校验器必须复用同一个运动学假设，否则校验说"不重叠"、仿真里却撞上了。
    """
    p = min(1.0, max(0.0, progress))
    return 0.5 * (1.0 - math.cos(math.pi * p))


def predict_pose(actor: Actor, t: float, lane_width: float) -> tuple[float, float, float]:
    """外推 actor 在 t 时刻的位姿 (x, y, theta)。

    跟驰行为在这里按匀速近似——纯几何校验不需要精确的 IDM 响应，
    需要的是"两车会走到哪里"，匀速已经是保守估计（真实跟驰只会更慢）。
    """
    speed = actor.speed
    if speed <= 0.0:
        return actor.x, actor.y, actor.theta

    if actor.behavior != "cut_in":
        return (
            actor.x + speed * math.cos(actor.theta) * t,
            actor.y + speed * math.sin(actor.theta) * t,
            actor.theta,
        )

    # 切入的前段是沿原车道直行，后段横向平移 + 可能减速。
    lane_change = max(0.25, actor.lane_change_seconds)
    before = min(t, actor.trigger)
    moving = max(0.0, t - actor.trigger)

    traveller = before + min(moving, lane_change)
    x = actor.x + speed * math.cos(actor.theta) * traveller
    y = actor.y + speed * math.sin(actor.theta) * traveller

    # 横向位移：起点为本车道，终点为 targetY。
    # 缺省规则与 C++ `ScenarioService::defaultCutInTargetY` 逐字对应：
    # 在中心车道则往左切一条，否则朝中心方向切一条。两处若不一致，
    # 校验会说"不重叠"而仿真里穿模——这正是校验器最不能出的错。
    if actor.target_y is not None:
        y_end = actor.target_y
    elif abs(actor.y) < 1e-6:
        y_end = -lane_width
    else:
        y_end = actor.y - math.copysign(lane_width, actor.y)

    progress = min(1.0, moving / lane_change) if moving > 0.0 else 0.0
    y = actor.y + (y_end - actor.y) * _cut_in_ratio(progress)

    theta = actor.theta
    return x, y, theta


def _overlaps(a: tuple[float, float, float, float, float],
              b: tuple[float, float, float, float, float]) -> bool:
    """两个轴对齐包围盒是否重叠。参数为 (x, y, theta, length, width)。

    用轴对齐近似（忽略航向）是有意的：车身偏转最多 10°，AABB 只会
    略微变大，不会漏报。宁可多报一次让人看一眼，也不要漏掉穿模。
    """
    ax, ay, _, al, aw = a
    bx, by, _, bl, bw = b
    return (abs(ax - bx) < (al + bl) * 0.5) and (abs(ay - by) < (aw + bw) * 0.5)


def check_semantics(scenario: Scenario) -> list[Issue]:
    """返回语义问题列表。空列表表示场景自洽。"""
    issues: list[Issue] = []
    dur = scenario.duration

    # ---- 1. 参与者基本几何 ----
    if not scenario.actors:
        issues.append(Issue("/actors", "场景至少要有一个交通参与者", "semantic"))

    ego_box = (0.0, 0.0, 0.0, EGO_LENGTH, EGO_WIDTH)

    for i, actor in enumerate(scenario.actors):
        path = f"/actors/{i}"
        length, width = _actor_size(actor)

        if actor.x <= 0.0 and actor.behavior != "constant":
            issues.append(Issue(
                f"{path}/x",
                f"{actor.behavior} 行为的参与者位于自车后方 {abs(actor.x):.1f} m，"
                "本场景只描述自车前方的交互",
                "semantic"))

        # 量程：远到雷达看不见的参与者等于不存在，会让失效搜索白跑。
        reach = math.hypot(actor.x, actor.y)
        if reach > scenario.sensor.lidar_range:
            issues.append(Issue(
                f"{path}/x",
                f"初始距离 {reach:.1f} m 超出雷达量程 {scenario.sensor.lidar_range:.0f} m，"
                "该参与者在感知中不可见",
                "semantic"))

        # 横向位置必须落在车道内：停在线上的车，行为语义不明确。
        if actor.type in ("car", "truck") and scenario.lane_width > 0.0:
            lane_pos = actor.y / scenario.lane_width
            offset = abs(lane_pos - round(lane_pos)) * scenario.lane_width
            if offset > scenario.lane_width * 0.3:
                issues.append(Issue(
                    f"{path}/y",
                    f"横向坐标 {actor.y:.2f} m 偏离最近车道中心 {offset:.2f} m，"
                    "车身压在车道线上",
                    "semantic"))

        if actor.type in ("car", "truck") and \
                abs(round(actor.y / scenario.lane_width)) >= scenario.lane_count:
            issues.append(Issue(
                f"{path}/y",
                f"横向坐标 {actor.y:.2f} m 超出 {scenario.lane_count} 条车道的范围",
                "semantic"))

        # ---- 2. 行为参数的内部一致性 ----
        if actor.behavior == "brake":
            if actor.brake_accel <= 0.0:
                issues.append(Issue(
                    f"{path}/brakeAccel",
                    "制动行为没有给出正的减速度，这辆车会一直匀速——"
                    "文件看着像\"急刹\"，跑起来什么也不发生",
                    "semantic"))
            if actor.trigger >= dur:
                issues.append(Issue(
                    f"{path}/trigger",
                    f"制动触发时刻 {actor.trigger:.1f} s 不早于场景时长 {dur:.1f} s，永远不会发生",
                    "semantic"))
            if actor.brake_accel > 0.0 and actor.speed > 0.0:
                stop_time = (actor.speed - actor.speed_floor) / actor.brake_accel
                if actor.trigger + stop_time > dur:
                    issues.append(Issue(
                        f"{path}/brakeAccel",
                        f"以 {actor.brake_accel:.1f} m/s² 从 {actor.speed:.1f} m/s 刹到 "
                        f"{actor.speed_floor:.1f} m/s 需 {stop_time:.1f} s，"
                        f"加上触发时刻 {actor.trigger:.1f} s 超出场景时长 {dur:.1f} s",
                        "hint"))

        if actor.behavior == "cut_in":
            if actor.trigger >= dur:
                issues.append(Issue(
                    f"{path}/trigger",
                    f"切入触发时刻 {actor.trigger:.1f} s 不早于场景时长 {dur:.1f} s，永远不会发生",
                    "semantic"))
            if actor.trigger + actor.lane_change_seconds > dur:
                issues.append(Issue(
                    f"{path}/laneChangeSeconds",
                    f"切入从 {actor.trigger:.1f} s 开始、耗时 {actor.lane_change_seconds:.1f} s，"
                    f"超出场景时长 {dur:.1f} s，变道会被截断在半途",
                    "semantic"))
            start_y = actor.y
            end_y = actor.target_y if actor.target_y is not None else None
            if end_y is not None:
                span = abs(end_y - start_y)
                if span < scenario.lane_width * 0.5:
                    issues.append(Issue(
                        f"{path}/targetY",
                        f"切入横向位移只有 {span:.2f} m，不足半个车道宽 "
                        f"{scenario.lane_width * 0.5:.2f} m，不构成变道",
                        "semantic"))
                if span > scenario.lane_width * 1.5:
                    issues.append(Issue(
                        f"{path}/targetY",
                        f"切入横向位移 {span:.2f} m 超过一个半车道宽，等于连跨两条车道",
                        "semantic"))

        # ---- 3. 初始位姿重叠 ----
        box = (actor.x, actor.y, actor.theta, length, width)
        if _overlaps(ego_box, box):
            issues.append(Issue(
                path,
                f"初始位姿与自车重叠（纵向 {actor.x:.1f} m、横向 {actor.y:.1f} m），"
                "开局即碰撞",
                "semantic"))

        for j in range(i):
            other = scenario.actors[j]
            o_len, o_wid = _actor_size(other)
            if _overlaps(box, (other.x, other.y, other.theta, o_len, o_wid)):
                issues.append(Issue(
                    path,
                    f"初始位姿与 /actors/{j} 重叠，两车叠在一起",
                    "semantic"))

    # ---- 4. 轨迹互穿（本轮新增的关键检查）----
    issues.extend(_check_trajectory_crossing(scenario))

    # ---- 5. 参数声明与绑定是否对得上 ----
    seen: set[str] = set()
    for k, param in enumerate(scenario.parameters):
        path = f"/parameters/{k}"
        if param.name in seen:
            issues.append(Issue(f"{path}/name", f"参数名重复：{param.name}", "semantic"))
        seen.add(param.name)
        if param.low > param.high:
            issues.append(Issue(
                path, f"参数 {param.name} 下界 {param.low} 大于上界 {param.high}", "semantic"))

    # 感知噪声是评估侧变量，不需要落在模板字段上；其余四个必须有落点。
    bindable = {"ego_speed", "friction"}
    if any(a.behavior in ("car_following", "brake", "constant") for a in scenario.actors):
        bindable.add("lead_gap")
    if any(a.behavior == "cut_in" for a in scenario.actors):
        bindable.add("cut_in_trigger")
    for name in seen:
        if name in ("ego_speed", "friction", "perception_noise"):
            continue
        if name not in bindable:
            issues.append(Issue(
                "/parameters",
                f"参数 {name} 在场景里没有可绑定的字段（已知绑定："
                f"{'、'.join(sorted(bindable | {'perception_noise'}))}）",
                "hint"))

    return issues


def _check_trajectory_crossing(scenario: Scenario) -> list[Issue]:
    """按轨迹外推检测参与者之间是否会互相穿过。

    只报"穿模"（包围盒重叠），不报"距离过近"——后者是失效搜索要找的
    目标，在校验阶段拦掉就把研究对象消灭了。
    """
    issues: list[Issue] = []
    actors = scenario.actors
    if len(actors) < 2:
        return issues

    sizes = [_actor_size(a) for a in actors]
    steps = int(scenario.duration / _PREDICT_DT) + 1

    for i in range(len(actors)):
        for j in range(i):
            first_hit: float | None = None
            for step in range(steps):
                t = step * _PREDICT_DT
                pi = predict_pose(actors[i], t, scenario.lane_width)
                pj = predict_pose(actors[j], t, scenario.lane_width)
                bi = (pi[0], pi[1], pi[2], sizes[i][0], sizes[i][1])
                bj = (pj[0], pj[1], pj[2], sizes[j][0], sizes[j][1])
                if _overlaps(bi, bj):
                    first_hit = t
                    break
            if first_hit is not None and first_hit > 1e-9:
                issues.append(Issue(
                    f"/actors/{i}",
                    f"按轨迹外推，与 /actors/{j} 在 t≈{first_hit:.1f} s 车身重叠"
                    "（一车从另一车身上穿过），场景几何不自洽",
                    "semantic"))
    return issues


def check_all(document: dict[str, Any] | Scenario) -> list[Issue]:
    """结构 + 语义全检。传 JSON 字典或 Scenario 均可。"""
    if isinstance(document, Scenario):
        scenario = document
        issues: list[Issue] = []
    else:
        issues = check_structure(document)
        if any(i.kind == "structure" for i in issues):
            # 结构都不对就别做语义推断，否则会因缺字段抛异常、
            # 报出一堆无关的噪声错误。
            return issues
        scenario = Scenario.from_json(document)

    issues.extend(check_semantics(scenario))
    return issues


def structure_ok(issues: list[Issue]) -> bool:
    return not any(i.kind == "structure" for i in issues)


def semantic_ok(issues: list[Issue]) -> bool:
    return not any(i.kind == "semantic" for i in issues)
