"""场景模板的 JSON Schema 与最小校验器。

这份 Schema 是**唯一的结构真相**，三个地方都从它派生：

  1. `validate.check_structure` 用它做结构校验；
  2. `nlu` 把它渲染成提示词里的字段说明，喂给语言模型；
  3. `generate` 用它判断回退模板缺哪些字段。

若各写一份，三者必然分叉——模型按提示词产出 A，校验器按 Schema 拒绝 A，
而回退模板又补了第三套字段，排错时根本分不清是谁错了。

本模块不依赖 `jsonschema` 包，原因与 `python/` 侧一致：毕设环境不该
因为装不上第三方库就跑不起来。这里只实现本项目用得到的子集：
type / required / minimum / maximum / enum / items / properties / additionalProperties。

Schema 故意保持"宽进"——可选字段给了默认值就不必出现。场景文件多由
手写或模型生成，缺一个可选字段就让整个加载失败，体验很差。
"""

from __future__ import annotations

import math
from typing import Any

#: 参与者行为枚举，与 C++ `BehaviorKind` 的字符串形式一一对应。
BEHAVIORS = ("constant", "car_following", "cut_in", "brake")

#: 参与者类型，与 C++ `EntityKind` 对应。
ACTOR_TYPES = ("car", "truck", "pedestrian", "cyclist")

#: 失效搜索的固定变量名。名字即字段绑定（见 `template.apply_parameters`），
#: 不允许模型自由发明——否则搜索器不知道该往哪里写值。
SEARCH_VARS = ("ego_speed", "lead_gap", "cut_in_trigger", "friction", "perception_noise")


def _num(minimum: float | None = None, maximum: float | None = None) -> dict[str, Any]:
    node: dict[str, Any] = {"type": "number"}
    if minimum is not None:
        node["minimum"] = minimum
    if maximum is not None:
        node["maximum"] = maximum
    return node


ACTOR_SCHEMA: dict[str, Any] = {
    "type": "object",
    "required": ["type", "x", "speed"],
    "properties": {
        "type": {"type": "string", "enum": list(ACTOR_TYPES)},
        "x": _num(-500.0, 500.0),
        "y": _num(-20.0, 20.0),
        "theta": _num(-math.pi, math.pi),
        "speed": _num(0.0, 60.0),
        "behavior": {"type": "string", "enum": list(BEHAVIORS)},
        "trigger": _num(0.0, 120.0),
        # 以下四个只在 behavior == cut_in 时才有意义，因此不设必填。
        "targetY": _num(-20.0, 20.0),
        "laneChangeSeconds": _num(0.5, 30.0),
        "brakeAccel": _num(0.0, 10.0),
        "speedFloor": _num(0.0, 60.0),
    },
    "additionalProperties": True,
}

PARAMETER_SCHEMA: dict[str, Any] = {
    "type": "object",
    "required": ["name", "min", "max"],
    "properties": {
        "name": {"type": "string"},
        "min": _num(),
        "max": _num(),
        "step": _num(1e-6, 1000.0),
    },
    "additionalProperties": True,
}

SCENARIO_SCHEMA: dict[str, Any] = {
    "type": "object",
    "required": ["id", "name", "actors"],
    "properties": {
        "id": {"type": "string"},
        "name": {"type": "string"},
        "laneCount": {"type": "integer", "minimum": 1, "maximum": 8},
        "laneWidth": _num(2.5, 5.0),
        "curvature": _num(-0.1, 0.1),
        "friction": _num(0.1, 1.2),
        "visibility": _num(0.1, 1.5),
        "durationSeconds": _num(1.0, 120.0),
        "sensor": {
            "type": "object",
            "properties": {
                "lidarRange": _num(5.0, 200.0),
                "lidarPointsPerMeter": _num(0.1, 100.0),
            },
            "additionalProperties": True,
        },
        "actors": {"type": "array", "items": ACTOR_SCHEMA, "minItems": 1, "maxItems": 64},
        "parameters": {"type": "array", "items": PARAMETER_SCHEMA, "maxItems": 16},
    },
    "additionalProperties": True,
}


class Issue:
    """一条校验问题。

    `path` 用 JSON Pointer 风格（`/actors/2/speed`），这样回填给语言模型的
    错误信息可以直接指到字段，模型才改得准；只报"场景不合法"等于没说。
    """

    __slots__ = ("path", "message", "kind")

    def __init__(self, path: str, message: str, kind: str = "structure") -> None:
        self.path = path
        self.message = message
        self.kind = kind        # structure | semantic

    def __str__(self) -> str:
        where = self.path or "<root>"
        return f"[{self.kind}] {where}: {self.message}"

    def as_dict(self) -> dict[str, str]:
        return {"kind": self.kind, "path": self.path, "message": self.message}


_TYPE_MAP = {
    "number": (int, float),
    "integer": int,
    "string": str,
    "array": list,
    "object": dict,
    "boolean": bool,
}


def _type_ok(value: Any, expected: str) -> bool:
    # JSON 里没有整数/浮点之分，但 Python 的 bool 是 int 的子类，必须先挡掉。
    if expected in ("number", "integer") and isinstance(value, bool):
        return False
    if expected == "integer" and isinstance(value, float):
        return float(value).is_integer()
    return isinstance(value, _TYPE_MAP[expected])


def validate_node(value: Any, schema: dict[str, Any], path: str = "") -> list[Issue]:
    """按 Schema 递归校验，返回问题列表（空列表即通过）。

    只覆盖本项目用到的关键字，遇到未知关键字直接忽略而不是报错——
    将来给 Schema 补充描述性字段时不会连带改校验器。
    """
    issues: list[Issue] = []

    expected = schema.get("type")
    if expected and not _type_ok(value, expected):
        issues.append(Issue(path, f"应为 {expected}，实际是 {type(value).__name__}"))
        return issues      # 类型都不对，后续约束没有意义

    if "enum" in schema and value not in schema["enum"]:
        allowed = "、".join(str(v) for v in schema["enum"])
        issues.append(Issue(path, f"取值 {value!r} 不在允许集合内（{allowed}）"))

    if isinstance(value, (int, float)) and not isinstance(value, bool):
        low = schema.get("minimum")
        high = schema.get("maximum")
        if low is not None and value < low - 1e-12:
            issues.append(Issue(path, f"{value} 小于下界 {low}"))
        if high is not None and value > high + 1e-12:
            issues.append(Issue(path, f"{value} 大于上界 {high}"))

    if isinstance(value, dict):
        for key in schema.get("required", ()):
            if key not in value:
                issues.append(Issue(f"{path}/{key}", "缺少必填字段"))
        props = schema.get("properties", {})
        for key, sub in props.items():
            if key in value:
                issues.extend(validate_node(value[key], sub, f"{path}/{key}"))

    if isinstance(value, list):
        items = schema.get("items")
        min_items = schema.get("minItems")
        max_items = schema.get("maxItems")
        if min_items is not None and len(value) < min_items:
            issues.append(Issue(path, f"至少需要 {min_items} 个元素，实际 {len(value)} 个"))
        if max_items is not None and len(value) > max_items:
            issues.append(Issue(path, f"最多 {max_items} 个元素，实际 {len(value)} 个"))
        if items:
            for index, element in enumerate(value):
                issues.extend(validate_node(element, items, f"{path}/{index}"))

    return issues


def check_structure(document: Any) -> list[Issue]:
    """校验一份场景 JSON（已解析为 Python 对象）的结构。"""
    return validate_node(document, SCENARIO_SCHEMA, "")


def describe_for_prompt() -> str:
    """把 Schema 压成人读/模型读的字段清单，供提示词使用。

    只列关键约束，不倾倒整份 JSON Schema：模型需要的是"有哪些字段、
    各自什么单位、取值范围大概多少"，冗长的 required/properties 层级
    反而会挤占上下文。
    """
    lines = [
        "顶层字段：",
        "  id (string)        场景标识，英文小写下划线",
        "  name (string)      中文名称",
        "  laneCount (int)    车道数 1-8，默认 2",
        "  laneWidth (double) 车道宽 m，默认 3.5",
        "  friction (double)  路面附着系数 0.1-1.2，默认 0.9",
        "  durationSeconds (double) 仿真时长 s，1-120，默认 30",
        "  actors (array)     交通参与者，至少 1 个",
        "  parameters (array) 可选，搜索变量声明",
        "",
        "actors 元素字段：",
        "  type           car | truck | pedestrian | cyclist",
        "  x, y, theta    初始位姿，自车在原点朝 +x；x 为纵向距离 m",
        "  speed          初速 m/s，0-60",
        "  behavior       constant（匀速）| car_following（跟驰）| cut_in（切入）| brake（制动）",
        "  trigger        行为触发时刻 s",
        "  cut_in 专用：targetY 目标横向坐标、laneChangeSeconds 平移时长、",
        "              brakeAccel 切入后减速度、speedFloor 减速下限",
        "  brake 专用：brakeAccel 减速度（必填，正值）、speedFloor 减速下限",
    ]
    return "\n".join(lines)
