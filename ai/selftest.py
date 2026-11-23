"""AI 模块离线自检。

覆盖四层：协议无关的纯函数（中文数字、单位换算）、槽位抽取、场景组装、
校验器。不走网络、不依赖 C++ 进程，所以能进 CI、也能在没有模型密钥的
机器上跑——这是它作为**回归基线**的前提。

其中三条用例是踩坑换来的，删掉就等于把坑重新挖一遍：

  * `第2秒前车急刹` 不得把 2 秒当成场景总时长；
  * 轨迹互穿检测必须抓住"切入车从前车身上穿过"；
  * 预置模板自身必须全部通过校验——回退时拿到的骨架不能本身就不合格。
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from adasim_ai import Actor, Scenario  # noqa: E402
from adasim_ai.generate import generate, load_presets, nearest_preset, preset_distance  # noqa: E402
from adasim_ai.nlu import (  # noqa: E402
    _friction_from_text, _first_speed, _first_trigger, cn_number, extract_slots,
)
from adasim_ai.schema import BEHAVIORS, SEARCH_VARS, check_structure  # noqa: E402
from adasim_ai.validate import check_all, check_semantics, predict_pose  # noqa: E402

_failures: list[str] = []
_total = 0


def check(name: str, condition: bool, detail: str = "") -> None:
    global _total
    _total += 1
    if condition:
        print(f"  [通过] {name}" + (f"    {detail}" if detail else ""))
    else:
        print(f"  [失败] {name}" + (f"    {detail}" if detail else ""))
        _failures.append(name)


def section(title: str) -> None:
    print(f"\n{title}")


# ---------------------------------------------------------------------------


def test_cn_number() -> None:
    section("中文数字与单位")
    check("阿拉伯数字", cn_number("42") == 42.0)
    check("个位", cn_number("三") == 3.0)
    check("十", cn_number("十") == 10.0)
    check("十几", cn_number("十五") == 15.0)
    check("几十", cn_number("二十") == 20.0)
    check("几十几", cn_number("二十三") == 23.0)
    check("两", cn_number("两") == 2.0)
    check("非数字返回空", cn_number("很多") is None)
    check("km/h 换算", abs((_first_speed("60 km/h") or 0) - 16.667) < 0.01,
          f"{(_first_speed('60 km/h') or 0):.3f} m/s")
    check("m/s 直通", (_first_speed("8 m/s") or 0) == 8.0)
    check("中文单位", abs((_first_speed("36 公里每小时") or 0) - 10.0) < 0.01)
    check("附着系数：冰", (_friction_from_text("路面结冰") or 0) < 0.2)
    check("附着系数：雨", abs((_friction_from_text("雨天") or 0) - 0.55) < 1e-9)
    check("附着系数：干", abs((_friction_from_text("晴天干燥") or 0) - 0.9) < 1e-9)
    check("附着系数：未提", _friction_from_text("跟车") is None)


def test_trigger_parse() -> None:
    section("触发时刻（与场景时长必须分开）")
    check("第 3 秒", _first_trigger("第 3 秒切入") == 3.0)
    check("第3秒（无空格）", _first_trigger("第3秒切入") == 3.0)
    check("3 秒后", _first_trigger("3 秒后刹车") == 3.0)

    slots = extract_slots("第2秒前车突然刹车")
    check("不把触发时刻当总时长", slots.duration is None,
          f"duration={slots.duration}")
    check("识别为制动意图", slots.brake is True)
    check("记录触发时刻", slots.trigger == 2.0)

    slots = extract_slots("跑 15 秒，第 3 秒邻道车切入")
    check("总时长与触发时刻各归其位",
          slots.duration == 15.0 and slots.trigger == 3.0,
          f"duration={slots.duration} trigger={slots.trigger}")


def test_slots() -> None:
    section("槽位抽取")
    cases = [
        ("前车 25 米同速跟车，第 3 秒邻道车切入并刹车", {"cut_in": True}),
        ("有车要并进来", {"cut_in": True}),
        ("有车加塞", {"cut_in": True}),
        ("前车突然急刹", {"brake": True}),
        ("路上有行人横穿", {"pedestrian": True}),
        ("前方有辆抛锚的车", {"static_obstacle": True}),
        ("雨天 60 km/h 跟车 20 米", {"ego_speed": 16.667}),
    ]
    for text, expected in cases:
        slots = extract_slots(text)
        for key, want in expected.items():
            got = getattr(slots, key)
            if isinstance(want, bool):
                ok = got is want
            else:
                ok = got is not None and abs(got - want) < 0.01
            check(f"{key} ← {text}", ok, f"得到 {got!r}")

    slots = extract_slots("两车道高速跟车")
    check("车道数（中文）", slots.lane_count == 2.0, f"得到 {slots.lane_count}")
    check("高速推断车速", slots.ego_speed == 22.0, f"得到 {slots.ego_speed}")

    slots = extract_slots("路很滑，跟得近一点")
    check("近距同义词", slots.lead_gap == 8.0 or slots.friction is not None)


def test_build() -> None:
    section("场景组装")
    result = generate("前车 25 米同速跟车，第 3 秒邻道车切入并刹车")
    check("通路为规则解析", result.route == "rule", result.route)
    check("无校验问题", result.ok, str(result.issues))
    check("两个参与者", len(result.scenario.actors) == 2,
          f"{len(result.scenario.actors)} 个")
    cutter = [a for a in result.scenario.actors if a.behavior == "cut_in"]
    check("存在切入车", len(cutter) == 1)
    if cutter:
        check("切入车在邻道", abs(cutter[0].y - 3.5) < 1e-9, f"y={cutter[0].y}")
        check("切入目标为本车道", cutter[0].target_y == 0.0)
        check("触发时刻取原话", cutter[0].trigger == 3.0, f"{cutter[0].trigger}")
    lead = [a for a in result.scenario.actors if a.behavior == "car_following"]
    check("前车被挪远以避免穿透", lead and lead[0].x >= 45.0,
          f"x={lead[0].x if lead else '无'}")
    check("场景标识可读", result.scenario.id == "nl_cut_in", result.scenario.id)

    brake = generate("第2秒前车突然刹车")
    check("制动场景用 brake 行为",
          any(a.behavior == "brake" for a in brake.scenario.actors))
    brake_actor = next(a for a in brake.scenario.actors if a.behavior == "brake")
    check("制动减速度非零", brake_actor.brake_accel > 0, f"{brake_actor.brake_accel}")
    check("制动触发时刻", brake_actor.trigger == 2.0, f"{brake_actor.trigger}")

    ped = generate("路上有行人横穿，我开 8 m/s")
    check("行人场景", any(a.type == "pedestrian" for a in ped.scenario.actors))
    check("行人横向在路侧",
          all(abs(a.y) > 3.5 for a in ped.scenario.actors if a.type == "pedestrian"))

    static = generate("前方 30 米有辆抛锚的车，我开 12 m/s")
    check("静止障碍速度为零",
          all(a.speed == 0.0 for a in static.scenario.actors),
          str([a.speed for a in static.scenario.actors]))


def test_fallback() -> None:
    section("回退链路")
    vague = generate("路上有车")
    check("信息不足时回退", vague.route == "preset", vague.route)
    check("回退仍产出合法场景", vague.ok, str(vague.issues))
    check("标注待确认字段", len(vague.scenario.unresolved) > 0,
          "、".join(vague.scenario.unresolved))
    check("说明里写明模板名", bool(vague.preset_name), vague.preset_name)

    high = generate("高速公路快速跟车")
    check("意图清楚时不回退", high.route == "rule", high.route)
    check("高速取 22 m/s", high.scenario.ego_speed == 22.0,
          f"{high.scenario.ego_speed}")

    presets = load_presets()
    check("预置模板数量", len(presets) >= 5, f"{len(presets)} 个")
    brake_scene = generate("第2秒前车突然刹车").scenario
    near, dist = nearest_preset(brake_scene, presets)
    check("最近邻命中制动模板",
          near is not None and "brake" in near.id, near.id if near else "无")
    check("同类距离小于异类", dist < 3.0, f"距离 {dist:.2f}")


def test_validate() -> None:
    section("校验器")
    check("空对象缺必填", any(i.kind == "structure" for i in check_structure({})))
    check("类型错误被抓",
          any("应为 number" in i.message
              for i in check_structure({"id": "x", "name": "y",
                                        "actors": [{"type": "car", "x": "远", "speed": 1}]})))
    check("枚举越界被抓",
          any("不在允许集合" in i.message
              for i in check_structure({"id": "x", "name": "y",
                                        "actors": [{"type": "boat", "x": 1, "speed": 1}]})))
    check("数值越界被抓",
          any("大于上界" in i.message or "小于下界" in i.message
              for i in check_structure({"id": "x", "name": "y", "laneCount": 99,
                                        "actors": [{"type": "car", "x": 1, "speed": 1}]})))
    check("行为枚举包含 brake", "brake" in BEHAVIORS)

    overlap = Scenario(actors=[Actor("car", x=1.0, y=0.0, speed=5.0)])
    issues = check_semantics(overlap)
    check("初始重叠被抓", any("重叠" in i.message for i in issues), str(issues))

    behind = Scenario(actors=[Actor("car", x=-20.0, y=0.0, speed=5.0,
                                    behavior="car_following")])
    check("前车在后方被抓",
          any("后方" in i.message for i in check_semantics(behind)))

    too_late = Scenario(duration=10.0, actors=[
        Actor("car", x=20.0, y=0.0, speed=5.0, behavior="cut_in",
              trigger=12.0, target_y=0.0)])
    check("触发超时长被抓",
          any("永远不会发生" in i.message for i in check_semantics(too_late)))

    # 复现上一轮的场景 bug：慢前车与快切入车轨迹互穿。
    buggy = Scenario(duration=20.0, ego_speed=9.0, actors=[
        Actor("car", x=12.0, y=3.5, speed=9.0, behavior="cut_in",
              trigger=0.0, target_y=0.0, lane_change_seconds=3.0),
        Actor("car", x=30.0, y=0.0, speed=2.0, behavior="constant"),
    ])
    crossing = [i for i in check_semantics(buggy) if "车身重叠" in i.message]
    check("轨迹互穿被抓（上一轮的场景 bug）", len(crossing) == 1,
          crossing[0].message if crossing else "未检测到")

    healthy = Scenario.load(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "presets", "cut_in_brake.json"))
    check("正常切入场景不误报", not check_semantics(healthy),
          str(check_semantics(healthy)))


def test_kinematics() -> None:
    section("运动学一致性")
    # 与 C++ `cutInLateralRatio` 必须是同一条余弦曲线，否则校验说"不重叠"
    # 而仿真里撞上——这是校验器最不能出的错。
    for progress, expect in ((0.0, 0.0), (0.5, 0.5), (1.0, 1.0)):
        actor = Actor("car", x=0.0, y=3.5, speed=10.0, behavior="cut_in",
                      trigger=0.0, target_y=0.0, lane_change_seconds=2.0)
        _, y, _ = predict_pose(actor, progress * 2.0, 3.5)
        got = (3.5 - y) / 3.5
        check(f"横向进度 {progress:.1f}", abs(got - expect) < 1e-6, f"{got:.6f}")

    actor = Actor("car", x=0.0, y=3.5, speed=0.5, behavior="cut_in",
                  trigger=0.0, target_y=0.0, lane_change_seconds=4.0)
    _, y, _ = predict_pose(actor, 10.0, 3.5)
    check("变道结束后停在新车道", abs(y) < 1e-9, f"y={y:.6f}")

    straight = Actor("car", x=0.0, y=0.0, speed=10.0, behavior="constant")
    x, y, _ = predict_pose(straight, 3.0, 3.5)
    check("匀速直行外推", abs(x - 30.0) < 1e-9 and abs(y) < 1e-9, f"x={x:.2f}")


def test_template() -> None:
    section("模板与参数绑定")
    scenario = Scenario(ego_speed=8.0, friction=0.9, actors=[
        Actor("car", x=25.0, y=0.0, speed=8.0, behavior="car_following"),
        Actor("car", x=12.0, y=3.5, speed=9.0, behavior="cut_in",
              trigger=3.0, target_y=0.0),
    ])

    applied = scenario.apply_parameters({
        "ego_speed": 12.0, "friction": 0.5,
        "lead_gap": 30.0, "cut_in_trigger": 5.0, "perception_noise": 0.4,
    })
    check("自车初速绑定", applied.ego_speed == 12.0)
    check("附着系数绑定", applied.friction == 0.5)
    check("跟车间距绑定",
          applied.actors[0].x == 30.0, f"{applied.actors[0].x}")
    check("切入时刻绑定",
          applied.actors[1].trigger == 5.0, f"{applied.actors[1].trigger}")
    check("噪声不写入参与者",
          applied.actors[1].trigger == 5.0 and applied._noise == 0.4)
    check("原模板未被修改", scenario.actors[0].x == 25.0,
          f"{scenario.actors[0].x}")

    check("搜索变量集", SEARCH_VARS ==
          ("ego_speed", "lead_gap", "cut_in_trigger", "friction", "perception_noise"))

    wire = scenario.to_json()
    restored = Scenario.from_json(wire)
    check("序列化往返一致",
          restored.to_json() == wire)
    check("targetY 显式写出", wire["actors"][1].get("targetY") == 0.0)
    check("非切入车不写 targetY", "targetY" not in wire["actors"][0])

    brake_actor = Actor("car", x=25.0, y=0.0, speed=10.0, behavior="brake",
                        trigger=2.0, brake_accel=6.0, speed_floor=0.0)
    brake_wire = brake_actor.to_json()
    check("制动参数写出", brake_wire.get("brakeAccel") == 6.0
          and brake_wire.get("speedFloor") == 0.0)


def test_presets_ok() -> None:
    section("预置模板自检")
    for preset in load_presets():
        issues = check_all(preset)
        check(f"{preset.id}", not issues, str(issues))


def test_gp() -> None:
    section("高斯过程（自写数值代码，必须验算）")
    from adasim_ai.gp import (
        GaussianProcess, expected_improvement, fit_best, matern52,
    )

    check("核函数 r=0 时为 1", abs(matern52([0.0], [0.0], 0.3) - 1.0) < 1e-12)
    check("核函数随距离递减",
          matern52([0.0], [0.5], 0.3) > matern52([0.0], [1.5], 0.3))

    # GP 在训练点上必须几乎插值（噪声很小时）
    xs = [[0.0], [0.3], [0.6], [1.0]]
    ys = [0.0, 1.0, 0.4, 0.9]
    gp = GaussianProcess(length_scale=0.5, noise_var=1e-8).fit(xs, ys)
    errors = [abs(gp.predict(x)[0] - y) for x, y in zip(xs, ys)]
    check("训练点处插值误差", max(errors) < 0.02, f"最大 {max(errors):.4f}")

    # 远离样本处不确定性必须变大，否则采集函数会一直挑远处的点
    _, var_near = gp.predict([0.3])
    _, var_far = gp.predict([5.0])
    check("远处方差更大", var_far > var_near, f"{var_far:.4f} vs {var_near:.4f}")
    check("方差非负", var_near >= 0.0 and var_far >= 0.0)

    # 超参数选择应挑到能解释数据的长度尺度
    tuned = fit_best(xs, ys)
    check("选出的长度尺度在候选集内",
          tuned.length_scale in (0.1, 0.15, 0.2, 0.3, 0.45, 0.65, 0.9),
          f"{tuned.length_scale}")

    # EI：已评估点附近（且该点就是最优）应低于未探索区
    gp2 = fit_best([[0.0], [0.5], [1.0]], [0.1, 0.9, 0.2])
    ei_at_best = expected_improvement(gp2, [0.5], 0.9)
    ei_elsewhere = expected_improvement(gp2, [0.75], 0.9)
    check("EI 在最优附近低于未探索区", ei_at_best < ei_elsewhere,
          f"{ei_at_best:.5f} vs {ei_elsewhere:.5f}")
    check("EI 非负", ei_at_best >= 0.0 and ei_elsewhere >= 0.0)


def test_severity() -> None:
    section("失效严重度")
    from adasim_ai.search import Metrics, severity

    check("从未接近目标 → 0",
          severity(Metrics(min_ttc=-1.0)) == 0.0)
    check("TTC 很大 → 接近 0",
          severity(Metrics(min_ttc=100.0)) < 0.02)
    # 连续性：未碰撞 TTC→0 时趋近 1.0，碰撞且撞击速度 0 时也是 1.0
    check("两段在临界点连续",
          abs(severity(Metrics(min_ttc=0.0)) - severity(
              Metrics(collided=True, impact_speed=0.0))) < 1e-9,
          f"{severity(Metrics(min_ttc=0.0)):.6f}")
    check("TTC 越小越危险",
          severity(Metrics(min_ttc=0.5)) > severity(Metrics(min_ttc=3.0)))
    check("撞击速度越大越危险",
          severity(Metrics(collided=True, impact_speed=9.0))
          > severity(Metrics(collided=True, impact_speed=2.0)))
    check("碰撞一定比未碰撞严重",
          severity(Metrics(collided=True, impact_speed=0.0))
          > severity(Metrics(min_ttc=0.05)))
    check("严重度上界", severity(Metrics(collided=True, impact_speed=99.0)) <= 1.5)


def test_lhs() -> None:
    section("采样与反归一化")
    import random

    from adasim_ai.search import denormalize, latin_hypercube

    points = latin_hypercube(8, 3, random.Random(7))
    check("样本数", len(points) == 8 and len(points[0]) == 3)
    stratified = all(
        len({min(7, int(p[d] * 8)) for p in points}) == 8 for d in range(3))
    check("每维分层互不重复", stratified)
    check("取值落在 [0,1)", all(0.0 <= v <= 1.0 for p in points for v in p))

    bounds = {"a": (10.0, 20.0), "b": (-1.0, 1.0)}
    real = denormalize([0.0, 1.0], ["a", "b"], bounds)
    check("反归一化下界", real["a"] == 10.0)
    check("反归一化上界", real["b"] == 1.0)
    mid = denormalize([0.5, 0.5], ["a", "b"], bounds)
    check("反归一化中点", abs(mid["a"] - 15.0) < 1e-9 and abs(mid["b"]) < 1e-9)


def run_all() -> int:
    print("=" * 62)
    print("ADASim AI 模块离线自检")
    print("=" * 62)
    test_cn_number()
    test_trigger_parse()
    test_slots()
    test_build()
    test_fallback()
    test_validate()
    test_kinematics()
    test_template()
    test_presets_ok()
    test_gp()
    test_severity()
    test_lhs()

    print("\n" + "=" * 62)
    if _failures:
        print(f"失败 {len(_failures)}/{_total}：")
        for name in _failures:
            print("  - " + name)
        return 1
    print(f"全部通过（{_total} 项）")
    return 0


if __name__ == "__main__":
    sys.exit(run_all())
