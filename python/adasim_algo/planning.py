"""规划与控制：跟驰决策 + 横向跟踪 + 执行器模型。

分工说明：
  * 规划负责"该以多快的速度、沿哪条路径走"，输出期望加速度与参考转角；
  * 控制负责"执行器能不能听话"，加入一阶滞后与限幅。

之所以把滞后放在这里而不是交给物理引擎，是因为真实车辆的转向执行器
是机电系统，不可能在 20 ms 内完成从 -0.3 rad 到 +0.3 rad 的跳变。
物理引擎只做运动学积分，不承担执行器建模。

坐标约定与 C++ 侧一致：世界坐标 x 向前、y 向左、theta 逆时针为正，
车辆转角 steer 为正时左转（theta 增大）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .perception import Track


@dataclass(slots=True)
class PlanningConfig:
    """可调参数集中在一处，便于实验章节做参数敏感性分析。"""

    cruise_speed: float = 11.0      # 期望车速，m/s
    time_gap: float = 1.4           # 期望车头时距，s
    min_gap: float = 2.0            # 静止最小车距，m
    accel_max: float = 2.5          # 舒适加速度上限，m/s^2
    decel_max: float = 6.0          # 制动能力上限，m/s^2（取正值）
    comfort_decel: float = 2.0      # IDM 内部舒适减速度，m/s^2
    ego_length: float = 4.5
    ego_width: float = 1.9
    corridor_margin: float = 0.35   # 车道占用判定的横向余量，m
    lookahead: float = 0.6          # 选前车时向前预测的时域，s
    steer_gain: float = 1.6         # Stanley 增益
    max_steer: float = 0.6          # 与 VehicleParams::maxSteer 一致
    steer_tau: float = 0.12         # 转向执行器一阶时间常数，s


@dataclass(slots=True)
class Plan:
    accel: float
    steer: float
    target_speed: float
    lead_id: int | None
    gap: float
    ttc: float
    reason: str


def predict_position(track: Track, horizon: float) -> tuple[float, float]:
    """CTRV 简化外推：航向按当前朝向保持，速度做矢量外推。

    完整 CTRV 需要角速度，而感知只给出朝向、没有可靠的 yaw rate 估计；
    在直道场景下朝向恒定，这个简化与完整形式等价。
    """
    return track.x + track.vx * horizon, track.y + track.vy * horizon


class Planner:
    """单帧决策。内部保存上一步的转角，用于执行器滞后。"""

    __slots__ = ("cfg", "_steer")

    def __init__(self, cfg: PlanningConfig | None = None) -> None:
        self.cfg = cfg or PlanningConfig()
        self._steer = 0.0

    def reset(self) -> None:
        self._steer = 0.0

    def plan(self, ego, tracks: list[Track], dt: float) -> Plan:
        cfg = self.cfg

        lead, gap, lead_speed = select_lead(ego, tracks, cfg)

        if lead is None:
            accel = _speed_control(ego.speed, cfg.cruise_speed, cfg)
            reason = "cruise"
            ttc = math.inf
        else:
            accel = idm_accel(ego.speed, cfg.cruise_speed, gap, lead_speed, cfg)
            reason = "follow"
            ttc = gap / (ego.speed - lead_speed) if ego.speed > lead_speed + 0.05 else math.inf
            if gap < 1.0:
                # 已经贴上了：IDM 的二次项会给出巨大的负值，这里单独限幅，
                # 避免日志里出现 -180 m/s^2 这种没有物理意义的读数。
                accel = -cfg.decel_max
                reason = "emergency"

        accel = max(-cfg.decel_max, min(cfg.accel_max, accel))
        steer = self._track_path(ego, dt)

        return Plan(
            accel=accel,
            steer=steer,
            target_speed=cfg.cruise_speed,
            lead_id=lead.id if lead is not None else None,
            gap=gap,
            ttc=ttc,
            reason=reason,
        )

    def _track_path(self, ego, dt: float) -> float:
        """Stanley 横向控制，参考路径取车道中心线 y = 0。

        参考路径之所以写死为 y = 0，是因为现有的两个场景模板都是沿 +x 的
        直道，ego 初始位置在原点。接入地图后这里换成"最近路径点 + 切线"，
        接口不变。
        """
        cfg = self.cfg
        cross_track = -ego.y                 # 横向偏差，正值表示偏右
        heading_error = -ego.theta           # 目标航向为 0

        # 低速时 atan(k*e/v) 会发散，分母加下限是 Stanley 的常规做法。
        speed = max(ego.speed, 2.0)
        desired = heading_error + math.atan(cfg.steer_gain * cross_track / speed)
        desired = max(-cfg.max_steer, min(cfg.max_steer, desired))

        # 一阶滞后：等价于转向电机的时间常数，dt 越小时越接近连续形式。
        alpha = 1.0 - math.exp(-dt / cfg.steer_tau) if cfg.steer_tau > 0.0 else 1.0
        self._steer += (desired - self._steer) * alpha
        return max(-cfg.max_steer, min(cfg.max_steer, self._steer))


def idm_accel(v: float, v0: float, gap: float, lead_speed: float,
              cfg: PlanningConfig) -> float:
    """Intelligent Driver Model。

        a = a_max * [1 - (v/v0)^4 - (s*/s)^2]
        s* = s0 + max(0, v*T + v*Δv / (2*sqrt(a_max*b)))
    """
    dv = v - lead_speed
    s_star = cfg.min_gap + max(
        0.0, v * cfg.time_gap + v * dv / (2.0 * math.sqrt(cfg.accel_max * cfg.comfort_decel))
    )
    free = 1.0 - (v / v0) ** 4 if v0 > 0.0 else 1.0
    s = max(gap, 0.1)
    return cfg.accel_max * (free - (s_star / s) ** 2)


def _speed_control(v: float, target: float, cfg: PlanningConfig) -> float:
    """无前车时的定速跟踪。比例系数取得保守，避免油门刹车来回切换。"""
    return max(-cfg.decel_max, min(cfg.accel_max, 0.8 * (target - v)))


def select_lead(ego, tracks: list[Track],
                cfg: PlanningConfig) -> tuple[Track | None, float, float]:
    """在自车走廊内选出最近的前车。

    判定条件是"占用自车走廊"，而不是"处于同一车道"：
    走廊宽度取自车宽加目标车宽的一半，再加余量。这样相邻车道上
    贴近车道线的车辆不会被误当成前车，而真正切入的目标一旦进入
    走廊就会被捕捉。

    这里有两套位置，用途必须分开，混用会引入系统性偏差：

      * **预测位置**（外推 lookahead）只用于判断"是否占用我的走廊"。
        切入车当前还在邻道，只有外推后才能看出它将要挡住我。
      * **当前观测位置**只用于计算车距。若车距也用预测位置，
        前车会被凭空前推 v·lookahead（4 m/s × 0.6 s ≈ 2.4 m），
        车距被高估，控制器就会不自觉地跟得更近。
    """
    cos_t = math.cos(ego.theta)
    sin_t = math.sin(ego.theta)

    best: Track | None = None
    best_gap = math.inf
    best_speed = 0.0

    for tr in tracks:
        # ---- 用预测位置判断走廊占用 ----
        px, py = predict_position(tr, cfg.lookahead)
        dxp = px - ego.x
        dyp = py - ego.y
        lxp = dxp * cos_t + dyp * sin_t
        lyp = -dxp * sin_t + dyp * cos_t

        if lxp <= 0.0:
            continue
        corridor = 0.5 * (cfg.ego_width + tr.width) + cfg.corridor_margin
        if abs(lyp) > corridor:
            continue

        # ---- 车距用当前位置 ----
        dx = tr.x - ego.x
        dy = tr.y - ego.y
        lx = dx * cos_t + dy * sin_t
        if lx <= 0.0:
            continue

        # 车距按"保险杠到保险杠"计算，而不是中心距：
        # 中心距减半长之和，才是 IDM 需要的净空。
        gap = lx - 0.5 * cfg.ego_length - 0.5 * tr.length
        if gap < best_gap:
            best = tr
            best_gap = gap
            # 前车速度投影到自车纵轴，横向速度不参与跟驰
            best_speed = tr.vx * cos_t + tr.vy * sin_t

    if best is None:
        return None, math.inf, 0.0
    return best, max(best_gap, 0.0), max(best_speed, 0.0)
