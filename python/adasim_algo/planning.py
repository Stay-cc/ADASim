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


def road_pose_at(segments: list, s: float) -> tuple[float, float, float]:
    """分段道路中心线在里程 s 处的世界位姿 (x, y, theta)。

    与 C++ 侧 roadPoseAt 同一套路径积分：从里程 0 的世界原点、朝 +x 出发，
    直道沿切线直线延伸，弯道走半径 R = 1/κ 的圆弧。交点切线连续。
    返回 (x, y, theta)；s 超出范围时沿末端切线外推。
    """
    x, y, theta = 0.0, 0.0, 0.0
    if not segments:
        return s, 0.0, 0.0

    cursor = 0.0
    for seg in segments:
        seg_start = float(seg["startS"])
        seg_end = float(seg["endS"])
        length = seg_end - seg_start
        if length <= 0.0:
            continue
        kappa = float(seg["curvature"])
        if s <= seg_end:
            run = s - cursor
            if abs(kappa) < 1e-9:
                x += run * math.cos(theta)
                y += run * math.sin(theta)
            else:
                r = 1.0 / kappa
                dtheta = kappa * run
                cx = x - r * math.sin(theta)
                cy = y + r * math.cos(theta)
                a0 = math.atan2(y - cy, x - cx)
                a1 = a0 + dtheta
                x = cx + r * math.cos(a1)
                y = cy + r * math.sin(a1)
                theta += dtheta
            return x, y, theta
        # 整段走完
        if abs(kappa) < 1e-9:
            x += length * math.cos(theta)
            y += length * math.sin(theta)
        else:
            r = 1.0 / kappa
            dtheta = kappa * length
            cx = x - r * math.sin(theta)
            cy = y + r * math.cos(theta)
            a0 = math.atan2(y - cy, x - cx)
            a1 = a0 + dtheta
            x = cx + r * math.cos(a1)
            y = cy + r * math.sin(a1)
            theta += dtheta
        cursor = seg_end
    # 超出全部段：沿末端切线外推
    run = s - cursor
    x += run * math.cos(theta)
    y += run * math.sin(theta)
    return x, y, theta


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
    wheel_base: float = 2.7         # 轴距，m；弯道前馈转角 δ = atan(L·κ)


@dataclass(slots=True)
class Plan:
    accel: float
    steer: float
    target_speed: float
    lead_id: int | None
    gap: float
    ttc: float
    reason: str
    # 本帧的参考路径偏差，供实验章节统计横向/航向误差 RMS。
    # 放在 Plan 里而不是另开一条测量通道：它就是控制器本帧看到的量，
    # 事后另算一遍反而可能和控制器用的口径不一致。
    lat_error: float = 0.0
    heading_error: float = 0.0


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

    def plan(self, ego, tracks: list[Track], dt: float,
             curvature: float = 0.0,
             road_segments: list | None = None,
             odometer: float = 0.0) -> Plan:
        cfg = self.cfg

        lead, gap, lead_speed = select_lead(ego, tracks, cfg)

        if lead is None:
            accel = _speed_control(ego.speed, cfg.cruise_speed, cfg)
            reason = "cruise"
            ttc = math.inf
            target_speed = cfg.cruise_speed
        else:
            accel = idm_accel(ego.speed, cfg.cruise_speed, gap, lead_speed, cfg)
            reason = "follow"
            # 跟驰稳态就是前车速度：目标车速取 min(巡航, 前车)，速度误差
            # 才有明确的参考量。若一律用巡航车速，跟一辆 4 m/s 的慢车时
            # 会持续显示 7 m/s 的"误差"，那是工况而不是跟踪质量。
            target_speed = min(cfg.cruise_speed, max(0.0, lead_speed))
            ttc = gap / (ego.speed - lead_speed) if ego.speed > lead_speed + 0.05 else math.inf
            if gap < 1.0:
                # 已经贴上了：IDM 的二次项会给出巨大的负值，这里单独限幅，
                # 避免日志里出现 -180 m/s^2 这种没有物理意义的读数。
                accel = -cfg.decel_max
                reason = "emergency"

        accel = max(-cfg.decel_max, min(cfg.accel_max, accel))
        steer, cross_track, heading_error = self._track_path(
            ego, dt, curvature, road_segments, odometer)

        return Plan(
            accel=accel,
            steer=steer,
            target_speed=target_speed,
            lead_id=lead.id if lead is not None else None,
            gap=gap,
            ttc=ttc,
            reason=reason,
            lat_error=cross_track,
            heading_error=heading_error,
        )

    def _track_path(self, ego, dt: float,
                    curvature: float = 0.0,
                    road_segments: list | None = None,
                    odometer: float = 0.0) -> tuple[float, float, float]:
        """Stanley 横向控制，参考路径由道路几何决定。

        分两种道路：
        * 单曲率场景（road_segments 为空）：直道参考 y = 0；弯道按圆弧，
          圆心在 (0, R)，R = 1/κ。
        * 分段道路（road_segments 非空）：用路径积分求"自车里程处的中心线
          位姿 (xc, yc, θc) 与当前段曲率 κc"，横向偏差取自车到中心线切线的
          有符号距离。这是分段地图正确性的关键——弯道段的圆心不在世界原点，
          沿用"圆心在原点"的圆弧会把参考路径画到错误位置，横向误差会假性
          爆炸（实测 RMS 28 m，而车其实跟得不错）。
        """
        cfg = self.cfg

        # 当前段曲率：单曲率场景就是 curvature；分段场景按里程查。
        if road_segments:
            cur = None
            for seg in road_segments:
                if seg["startS"] <= odometer < seg["endS"]:
                    cur = seg
                    break
            kappa = float(cur["curvature"]) if cur else 0.0
        else:
            kappa = curvature

        if road_segments:
            # 分段道路：参考是"里程处的中心线点 + 切线"。
            xc, yc, theta_c = road_pose_at(road_segments, odometer)
            dx = ego.x - xc
            dy = ego.y - yc
            # 横向偏差：自车相对切线的有符号距离。切线方向 (cosθc, sinθc)，
            # 左法向为 (-sinθc, cosθc)。正值表示偏左。
            cross_track_left = -math.sin(theta_c) * dx + math.cos(theta_c) * dy
            # Stanley 口径"正值偏右"，取反。
            cross_track = -cross_track_left
            heading_error = math.atan2(math.sin(theta_c - ego.theta),
                                       math.cos(theta_c - ego.theta))
        elif abs(curvature) < 1e-9:
            cross_track = -ego.y                 # 横向偏差，正值表示偏右
            heading_error = -ego.theta           # 目标航向为 0
        else:
            radius = 1.0 / curvature             # 有符号：正为左弯
            dx = ego.x - 0.0
            dy = ego.y - radius
            dist = math.hypot(dx, dy)
            # 偏差量取"相对目标半径的外扩量"，再按弯向换算成 Stanley 的
            # "正值偏右"口径。这里不能写 copysign(dist - |R|, κ)：copysign
            # 只取第二个参数的符号，结果与偏差大小无关，内外侧会被判反——
            # 表现为自车越偏越往外打方向，几十米后就甩出路面。
            cross_track = math.copysign(1.0, curvature) * (dist - abs(radius))
            tangent = math.atan2(dy, dx) + math.copysign(0.5 * math.pi, curvature)
            heading_error = math.atan2(math.sin(tangent - ego.theta),
                                       math.cos(tangent - ego.theta))

        # 低速时 atan(k*e/v) 会发散，分母加下限是 Stanley 的常规做法。
        speed = max(ego.speed, 2.0)
        desired = heading_error + math.atan(cfg.steer_gain * cross_track / speed)
        # 曲率前馈：稳态在弯道上需要的转角是 atan(L·κ)，不给这一项，
        # 纯反馈必须靠"持续偏离参考线"攒出误差才有转角，稳态偏差降不下来。
        desired += math.atan(cfg.wheel_base * kappa)
        desired = max(-cfg.max_steer, min(cfg.max_steer, desired))

        # 一阶滞后：等价于转向电机的时间常数，dt 越小时越接近连续形式。
        alpha = 1.0 - math.exp(-dt / cfg.steer_tau) if cfg.steer_tau > 0.0 else 1.0
        self._steer += (desired - self._steer) * alpha
        steer = max(-cfg.max_steer, min(cfg.max_steer, self._steer))
        # 返回的是"本帧控制增益之前的参考偏差"，也就是跟踪误差本身；
        # 若返回 desired 或 steer，量纲会混进增益里，统计出来不是误差。
        return steer, abs(cross_track), heading_error


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
