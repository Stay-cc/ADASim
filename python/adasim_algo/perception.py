"""感知：点云聚类 → 目标框拟合 → 帧间关联 → 速度估计。

链路刻意从**点云**起步，而不是直接读场景真值。理由是这个平台的设计
目标之一就是"在轻量二维平台上保留传感器级观测能力"：如果感知模块
直接消费真值障碍物，那么"全链路闭环"就只剩下规划与控制两环，
点云渲染也就退化成了纯装饰。

四个步骤各自的取舍：

1. 聚类：栅格加速的 DBSCAN。纯 Python 下朴素 O(n²) 在十万点上不可用，
   而栅格把"找邻域"从全表扫描降为九个格子。eps 取 0.9 m，略大于
   雷达轮廓采样间距（1/1.5 ≈ 0.67 m），保证同一条边上的点必然连通。

2. 框拟合：对簇内点做 PCA，主轴即车体朝向。轮廓点云是矩形外框，
   PCA 主轴与矩形长边重合，因此 length 取主轴跨度、width 取次轴跨度。
   矩形轮廓的跨度恰好等于真实长宽，不需要再补偿采样端点误差。

3. 关联：贪心最近邻 + 距离门限。场景里目标数量少、间距大，
   匈牙利算法在这里没有收益，反而增加实现与调试成本。

4. 测速：α-β 滤波，位置用 α 修正、速度用 β 修正。不用差分测速，
   是因为聚类质心会因采样点增减而抖动，差分会把抖动放大成速度噪声。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .protocol import CloudFrame

#: 邻域半径，m。
#:
#: 下限由采样间距决定，而不是由 pointsPerMeter 直接决定：雷达沿每条边
#: 取 int(边长/间距) 段，短边会被向下取整放大点距。1.9 m 的车宽在
#: 1.5 点/m 下只分成 2 段，实际点距 0.95 m，比名义间距 0.67 m 大得多。
#: 因此 eps 必须大于 0.95。
#:
#: 上限由相邻车道的最小边缘间距决定：3.5 - 1.9 = 1.6 m。eps 若接近 1.6，
#: 并排两车会被单链连通成一个目标。
#: 取 1.15，落在 (0.95, 1.6) 区间内。
CLUSTER_EPS = 1.15
#: 少于该点数的簇判为噪声。
CLUSTER_MIN_POINTS = 4
#: 纯 Python 的处理上限。超过则按步长抽稀，避免单帧耗时吃掉整个节拍。
MAX_POINTS = 20000


@dataclass(slots=True)
class Detection:
    """单帧检测结果：一个带朝向的矩形框。"""

    x: float
    y: float
    theta: float
    length: float
    width: float
    point_count: int = 0


@dataclass(slots=True)
class Track:
    """跨帧目标。速度由滤波器给出，不是位置差分。"""

    id: int
    x: float
    y: float
    theta: float
    vx: float
    vy: float
    length: float
    width: float
    hits: int = 1
    misses: int = 0
    last_t: float = 0.0

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    @property
    def confirmed(self) -> bool:
        """连续命中若干帧才对外输出，抑制单帧虚假目标。"""
        return self.hits >= 3


@dataclass(slots=True)
class PerceptionStats:
    points: int = 0
    decimated: int = 0
    clusters: int = 0
    tracks: int = 0
    elapsed_ms: float = 0.0


# ---------------------------------------------------------------- 聚类


def cluster_points(xs, ys, eps: float = CLUSTER_EPS,
                   min_points: int = CLUSTER_MIN_POINTS) -> list[list[int]]:
    """栅格加速的 DBSCAN，返回每个簇的点索引列表。

    只实现"密度可达"的连通分量部分，不区分核心点与边界点的传播顺序——
    对本题（矩形轮廓、形状规整）而言，连通分量已经等价于目标。
    """
    n = len(xs)
    if n == 0:
        return []

    inv = 1.0 / eps
    eps_sq = eps * eps
    grid: dict[tuple[int, int], list[int]] = {}
    for i in range(n):
        key = (math.floor(xs[i] * inv), math.floor(ys[i] * inv))
        bucket = grid.get(key)
        if bucket is None:
            grid[key] = [i]
        else:
            bucket.append(i)

    labels = [-1] * n
    clusters: list[list[int]] = []

    for seed in range(n):
        if labels[seed] != -1:
            continue
        label = len(clusters)
        labels[seed] = label
        stack = [seed]
        members: list[int] = []

        while stack:
            cur = stack.pop()
            members.append(cur)
            cx = math.floor(xs[cur] * inv)
            cy = math.floor(ys[cur] * inv)
            for gx in (cx - 1, cx, cx + 1):
                for gy in (cy - 1, cy, cy + 1):
                    bucket = grid.get((gx, gy))
                    if bucket is None:
                        continue
                    for j in bucket:
                        if labels[j] != -1:
                            continue
                        dx = xs[j] - xs[cur]
                        dy = ys[j] - ys[cur]
                        if dx * dx + dy * dy <= eps_sq:
                            labels[j] = label
                            stack.append(j)

        if len(members) >= min_points:
            clusters.append(members)
        else:
            # 噪声点必须把标签还回去，否则后续种子不会重访它们。
            for m in members:
                labels[m] = -1

    return clusters


def fit_box(xs, ys, members: list[int],
            prev_theta: float | None = None) -> Detection | None:
    """对簇内点做主成分分析，拟合带朝向的矩形框。"""
    m = len(members)
    if m < 3:
        return None

    sx = sy = 0.0
    for i in members:
        sx += xs[i]
        sy += ys[i]
    cx = sx / m
    cy = sy / m

    sxx = syy = sxy = 0.0
    for i in members:
        dx = xs[i] - cx
        dy = ys[i] - cy
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy

    # 二阶矩主方向：theta = 0.5 * atan2(2*sxy, sxx - syy)
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    c = math.cos(theta)
    s = math.sin(theta)

    lo_l = hi_l = None
    lo_w = hi_w = None
    for i in members:
        dx = xs[i] - cx
        dy = ys[i] - cy
        along = dx * c + dy * s
        across = -dx * s + dy * c
        if lo_l is None:
            lo_l = hi_l = along
            lo_w = hi_w = across
        else:
            if along < lo_l:
                lo_l = along
            elif along > hi_l:
                hi_l = along
            if across < lo_w:
                lo_w = across
            elif across > hi_w:
                hi_w = across

    length = hi_l - lo_l
    width = hi_w - lo_w
    if length <= 0.0 or width <= 0.0:
        return None

    # 主轴方向存在 ±π 二义性。车辆长度大于宽度，若拟合出的"长边"其实
    # 是横向，就把朝向转 90°；再按上一帧朝向消除 π 翻转，避免朝向跳变。
    if width > length:
        length, width = width, length
        theta += math.pi * 0.5

    theta = math.atan2(math.sin(theta), math.cos(theta))
    if prev_theta is not None:
        alt = theta + math.pi
        if abs(_wrap(alt - prev_theta)) < abs(_wrap(theta - prev_theta)):
            theta = alt
    theta = _wrap(theta)

    return Detection(cx, cy, theta, length, width, m)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


# ---------------------------------------------------------------- 跟踪


@dataclass(slots=True)
class Tracker:
    """贪心最近邻关联 + α-β 滤波。"""

    gate: float = 3.5            # 关联门限，m
    max_misses: int = 6          # 连续丢失多少帧后删除航迹
    alpha: float = 0.65          # 位置修正系数
    beta: float = 0.22           # 速度修正系数
    max_speed: float = 45.0

    _tracks: list[Track] = field(default_factory=list)
    _next_id: int = 1

    def update(self, detections: list[Detection], now: float) -> list[Track]:
        # 推进到当前时刻，同时把每个航迹的 dt 记下来。
        # 这一步必须在关联前完成（否则关联用的是过期的位置），
        # 但不能就地修改 last_t 之后再算 dt——那样 dt 恒为 0，
        # 滤波器会退化成"位置直接赋值、速度永远不更新"。
        steps: list[float] = []
        for tr in self._tracks:
            dt = now - tr.last_t
            if dt > 0.0:
                tr.x += tr.vx * dt
                tr.y += tr.vy * dt
            steps.append(dt if dt > 0.0 else 0.0)
            tr.last_t = now

        matched_tracks: set[int] = set()
        matched_dets: set[int] = set()

        # 贪心：每次取全局最小距离的一对。目标数少，代价可忽略，
        # 且比"按航迹顺序各取最近"更不容易出现交叉错配。
        pairs: list[tuple[float, int, int]] = []
        for ti, tr in enumerate(self._tracks):
            for di, det in enumerate(detections):
                d = math.hypot(det.x - tr.x, det.y - tr.y)
                if d <= self.gate:
                    pairs.append((d, ti, di))
        pairs.sort()

        for _d, ti, di in pairs:
            if ti in matched_tracks or di in matched_dets:
                continue
            matched_tracks.add(ti)
            matched_dets.add(di)
            self._apply(self._tracks[ti], detections[di], steps[ti])

        survivors: list[Track] = []
        for ti, tr in enumerate(self._tracks):
            if ti in matched_tracks:
                survivors.append(tr)
                continue
            tr.misses += 1
            if tr.misses <= self.max_misses:
                survivors.append(tr)

        for di, det in enumerate(detections):
            if di in matched_dets:
                continue
            survivors.append(
                Track(
                    id=self._next_id,
                    x=det.x,
                    y=det.y,
                    theta=det.theta,
                    vx=0.0,
                    vy=0.0,
                    length=det.length,
                    width=det.width,
                    last_t=now,
                )
            )
            self._next_id += 1

        self._tracks = survivors
        return survivors

    def _apply(self, tr: Track, det: Detection, dt: float) -> None:
        if dt <= 0.0:
            tr.x, tr.y = det.x, det.y
            tr.theta = det.theta
            return

        rx = det.x - tr.x
        ry = det.y - tr.y
        tr.x += self.alpha * rx
        tr.y += self.alpha * ry

        # β/dt 在 20 ms 步长下约等于 11，残差 0.05 m 对应 0.55 m/s 修正，
        # 量级合适：既能在 5 帧（0.1 s）内锁定新目标的速度，
        # 又不会把聚类质心的抖动当成真实机动。
        tr.vx += self.beta * rx / dt
        tr.vy += self.beta * ry / dt
        speed = math.hypot(tr.vx, tr.vy)
        if speed > self.max_speed:
            k = self.max_speed / speed
            tr.vx *= k
            tr.vy *= k

        tr.theta = det.theta
        tr.length = 0.7 * tr.length + 0.3 * det.length
        tr.width = 0.7 * tr.width + 0.3 * det.width
        tr.hits += 1
        tr.misses = 0


# ---------------------------------------------------------------- 门面


class Perception:
    """把点云变成可用航迹，并记录处理耗时与中间量。

    时间基准取**点云自带的仿真时间**，而不是墙钟。原因不是洁癖：
    状态帧会成簇到达（一次 select 可能取出好几帧），若用墙钟做 dt，
    两帧之间可能只差 0.2 ms，而 β/dt 会把几厘米的残差放大成几十 m/s
    的速度尖峰，航迹随即被外推到自车跟前，触发一次假紧急制动。
    用仿真时间既与传感器节拍一致，也天然免疫这种到达抖动。
    """

    __slots__ = ("tracker", "stats", "_last_frame", "_last_tracks")

    def __init__(self) -> None:
        self.tracker = Tracker()
        self.stats = PerceptionStats()
        self._last_frame = -1
        self._last_tracks: list[Track] = []

    def reset(self) -> None:
        """清空航迹与缓存。回溯后必须调用，否则新时间轴会继承旧航迹。"""
        self.tracker._tracks.clear()
        self.tracker._next_id = 1
        self._last_frame = -1
        self._last_tracks = []

    def process(self, cloud: CloudFrame | None) -> list[Track]:
        import time as _time

        if cloud is None:
            return [t for t in self.tracker._tracks if t.confirmed]

        # 同一帧点云被多帧状态复用时不重复喂给滤波器：
        # 重复更新等价于"用同一观测把 dt 压到 0"，同样会放大速度。
        if cloud.frame == self._last_frame:
            return self._last_tracks

        begin = _time.perf_counter()
        stats = PerceptionStats()
        detections: list[Detection] = []

        xs, ys = cloud.xs, cloud.ys
        stats.points = len(xs)
        if stats.points > MAX_POINTS:
            # 抽稀是权宜之计：纯 Python 逐点处理十万点会到秒级，
            # 直接拖垮 20 ms 的节拍。向量化后端（numpy）落地后可去掉。
            stride = (stats.points + MAX_POINTS - 1) // MAX_POINTS
            xs = xs[::stride]
            ys = ys[::stride]
            stats.decimated = stride

        clusters = cluster_points(xs, ys)
        stats.clusters = len(clusters)

        prev_by_pos: dict[tuple[int, int], float] = {}
        for tr in self.tracker._tracks:
            prev_by_pos[(int(tr.x), int(tr.y))] = tr.theta

        for members in clusters:
            det = fit_box(xs, ys, members)
            if det is None:
                continue
            # 朝向二义性消解放在拟合之后做：PCA 只算一次，
            # 再用最近航迹的朝向决定要不要翻转 π。
            nearest = _nearest_theta(prev_by_pos, det.x, det.y)
            if nearest is not None:
                alt = det.theta + math.pi
                if abs(_wrap(alt - nearest)) < abs(_wrap(det.theta - nearest)):
                    det.theta = _wrap(alt)
            detections.append(det)

        tracks = self.tracker.update(detections, cloud.sim_time)
        confirmed = [t for t in tracks if t.confirmed]

        stats.tracks = len(confirmed)
        stats.elapsed_ms = (_time.perf_counter() - begin) * 1000.0
        self.stats = stats
        self._last_frame = cloud.frame
        self._last_tracks = confirmed
        return confirmed


def _nearest_theta(table: dict[tuple[int, int], float], x: float, y: float):
    if not table:
        return None
    best = min(table.items(), key=lambda kv: (kv[0][0] - x) ** 2 + (kv[0][1] - y) ** 2)
    (bx, by), theta = best
    if (bx - x) ** 2 + (by - y) ** 2 <= 9.0:
        return theta
    return None
