"""场景模板的 Python 镜像，以及搜索变量到字段的绑定。

字段与 C++ `domain/scenario/ScenarioTemplate.h` 逐一对齐，JSON 键名与
`JsonScenarioRepository` 的读写保持一致。这里不做"更 Pythonic"的改造
（比如把 `x/y/theta` 收进 `pose` 子对象），因为两侧要读写同一批文件，
任何形态差异最终都会变成一类只在跨语言时暴露的 bug。

**参数绑定**是搜索器与生成器的接缝：搜索器只认识五个变量名，模板负责把
数值写到正确的位置。绑定用固定约定而不是配置文件——变量集在开题里已经
写死，多一层配置只会多一处不一致。
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from typing import Any

from .schema import SEARCH_VARS


@dataclass
class Actor:
    type: str = "car"
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    speed: float = 0.0
    behavior: str = "constant"
    trigger: float = 0.0
    target_y: float | None = None
    lane_change_seconds: float = 4.0
    brake_accel: float = 0.0
    speed_floor: float = 0.0

    def to_json(self) -> dict[str, Any]:
        # targetY 只在显式给出时写出：C++ 侧用"键是否存在"区分
        # "没写（走车道默认值）"与"写了 0（明确要求切到中心车道）"。
        out: dict[str, Any] = {
            "type": self.type,
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "theta": round(self.theta, 6),
            "speed": round(self.speed, 4),
            "behavior": self.behavior,
            "trigger": round(self.trigger, 4),
        }
        if self.target_y is not None:
            out["targetY"] = round(self.target_y, 4)
        # 制动参数对 cut_in 与 brake 都有意义：切入是"别进来同时踩刹车"，
        # 制动是"前车突然刹停"。只在 cut_in 时写出会让 brake 场景丢参数。
        if self.behavior in ("cut_in", "brake"):
            out["brakeAccel"] = round(self.brake_accel, 4)
            out["speedFloor"] = round(self.speed_floor, 4)
        if self.behavior == "cut_in":
            out["laneChangeSeconds"] = round(self.lane_change_seconds, 4)
        return out

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> "Actor":
        return cls(
            type=str(data.get("type", "car")),
            x=float(data.get("x", 0.0)),
            y=float(data.get("y", 0.0)),
            theta=float(data.get("theta", 0.0)),
            speed=float(data.get("speed", 0.0)),
            behavior=str(data.get("behavior", "constant")),
            trigger=float(data.get("trigger", 0.0)),
            target_y=float(data["targetY"]) if isinstance(data.get("targetY"), (int, float)) else None,
            lane_change_seconds=float(data.get("laneChangeSeconds", 4.0)),
            brake_accel=float(data.get("brakeAccel", 0.0)),
            speed_floor=float(data.get("speedFloor", 0.0)),
        )


@dataclass
class Parameter:
    name: str
    low: float
    high: float
    step: float = 0.1

    def to_json(self) -> dict[str, Any]:
        return {"name": self.name, "min": self.low, "max": self.high, "step": self.step}

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> "Parameter":
        return cls(
            name=str(data.get("name", "")),
            low=float(data.get("min", 0.0)),
            high=float(data.get("max", 1.0)),
            step=float(data.get("step", 0.1)),
        )


@dataclass
class Sensor:
    lidar_range: float = 50.0
    lidar_points_per_meter: float = 1.5


@dataclass
class Scenario:
    id: str = "scenario"
    name: str = "未命名场景"
    lane_count: int = 2
    lane_width: float = 3.5
    curvature: float = 0.0
    friction: float = 0.9
    visibility: float = 1.0
    duration: float = 30.0
    ego_speed: float = 8.0
    sensor: Sensor = field(default_factory=Sensor)
    actors: list[Actor] = field(default_factory=list)
    parameters: list[Parameter] = field(default_factory=list)
    #: 生成器标注的"需要人工确认"的字段路径，例如 `["/actors/0/speed"]`。
    #: 不写进 JSON 而是单独带出，避免污染场景语义。
    unresolved: list[str] = field(default_factory=list)
    #: 感知噪声强度（距离观测标准差，m）。不参与 JSON 序列化——
    #: 它是评估侧的观测扰动，不是场景本身的物理属性。
    _noise: float = 0.0

    # ---- 序列化 ----

    def to_json(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "id": self.id,
            "name": self.name,
            "laneCount": self.lane_count,
            "laneWidth": self.lane_width,
            "curvature": self.curvature,
            "friction": self.friction,
            "visibility": self.visibility,
            "durationSeconds": self.duration,
            "egoSpeed": self.ego_speed,
            "sensor": {
                "lidarRange": self.sensor.lidar_range,
                "lidarPointsPerMeter": self.sensor.lidar_points_per_meter,
            },
            "actors": [a.to_json() for a in self.actors],
        }
        if self.parameters:
            out["parameters"] = [p.to_json() for p in self.parameters]
        return out

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> "Scenario":
        sensor_raw = data.get("sensor") or {}
        return cls(
            id=str(data.get("id", "scenario")),
            name=str(data.get("name", "未命名场景")),
            lane_count=int(data.get("laneCount", 2)),
            lane_width=float(data.get("laneWidth", 3.5)),
            curvature=float(data.get("curvature", 0.0)),
            friction=float(data.get("friction", 0.9)),
            visibility=float(data.get("visibility", 1.0)),
            duration=float(data.get("durationSeconds", 30.0)),
            ego_speed=float(data.get("egoSpeed", 8.0)),
            sensor=Sensor(
                lidar_range=float(sensor_raw.get("lidarRange", 50.0)),
                lidar_points_per_meter=float(sensor_raw.get("lidarPointsPerMeter", 1.5)),
            ),
            actors=[Actor.from_json(a) for a in data.get("actors", []) if isinstance(a, dict)],
            parameters=[Parameter.from_json(p) for p in data.get("parameters", []) if isinstance(p, dict)],
        )

    def copy(self) -> "Scenario":
        return Scenario.from_json(self.to_json())

    # ---- 参数绑定 ----

    def search_bounds(self) -> dict[str, tuple[float, float]]:
        """从模板里抽取搜索变量的上下界。

        缺失的变量不参与搜索——这比给它补一个默认区间更安全：
        凭空补区间会让搜索器去调一个模板根本不支持的维度，白烧仿真预算。
        """
        bounds: dict[str, tuple[float, float]] = {}
        for p in self.parameters:
            if p.name in SEARCH_VARS and p.low <= p.high:
                bounds[p.name] = (p.low, p.high)
        return bounds

    def apply_parameters(self, values: dict[str, float]) -> "Scenario":
        """返回把搜索变量写入后的**新**模板（不改自身）。

        返回副本而非原地修改，是因为搜索器每轮都要从基线模板派生一个
        候选，原地改会把上一轮的取值带进下一轮。
        """
        out = self.copy()
        for name, value in values.items():
            if name == "ego_speed":
                out.ego_speed = float(value)
            elif name == "friction":
                out.friction = float(value)
            elif name == "lead_gap":
                # 前车的纵向位置。制动场景里前车是 brake 行为，
                # 只认 car_following 会让这类场景的间距无法被搜索。
                lead = (out._first_actor_of("car_following")
                        or out._first_actor_of("brake")
                        or out._first_actor_of("constant"))
                if lead is not None:
                    lead.x = float(value)
            elif name == "cut_in_trigger":
                cutter = out._first_actor_of("cut_in")
                if cutter is not None:
                    cutter.trigger = float(value)
            elif name == "perception_noise":
                # 噪声不在模板里落盘，它是评估侧的观测扰动强度。
                # 由 `_noise` 携带到批量运行的参数中。
                out._noise = float(value)
        return out

    def _first_actor_of(self, behavior: str) -> Actor | None:
        for a in self.actors:
            if a.behavior == behavior:
                return a
        return None

    # ---- 文件读写 ----

    def save(self, path: str) -> None:
        parent = os.path.dirname(os.path.abspath(path))
        os.makedirs(parent, exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(self.to_json(), fh, ensure_ascii=False, indent=2)
            fh.write("\n")

    @classmethod
    def load(cls, path: str) -> "Scenario":
        with open(path, "r", encoding="utf-8") as fh:
            return cls.from_json(json.load(fh))


def lane_center(lane_index: int, lane_width: float) -> float:
    """第 `lane_index` 条车道的中心横向坐标。

    自车所在车道（index 0）的中心是 y=0，向右为正。这个约定同时被
    生成器（算出切入目标）与校验器（判断是否压线）使用。
    """
    return lane_index * lane_width
