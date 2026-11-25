"""算法进程：事件循环、收发、统计上报。

刻意采用**单线程事件循环**，不开 RX/TX 线程。原因有三：

  1. 点云与状态必须在一起才能算控制量，分线程反而要加锁或拷贝；
  2. 单线程下"收到一帧状态 → 算一次 → 回一条指令"的因果顺序天然成立，
     不会出现指令与状态错配；
  3. 调度开销可控，20 ms 节拍里单帧计算只占一小部分。

这与底座侧"算绘分离"是同一套思想的两面：底座把渲染与物理分开，
算法侧则把"等待网络"与"计算"合成一个循环，两边都不靠加锁解决竞争。
"""

from __future__ import annotations

import math
import selectors
import socket
import time

from .perception import Perception, Track
from .planning import Planner, PlanningConfig
from .protocol import (
    CloudFrame,
    StateFrame,
    CloudReassembler,
    FrameDecoder,
    encode_command,
    parse_state,
)


class AlgoNode:
    """连接 ADASim 底座并按帧节拍闭环。"""

    def __init__(self, host: str = "127.0.0.1", tcp_port: int = 45100,
                 udp_port: int = 45101, use_truth: bool = False,
                 verbose: bool = True, planning_cfg: PlanningConfig | None = None,
                 warmup_seconds: float = 3.0):
        self.host = host
        self.tcp_port = tcp_port
        self.udp_port = udp_port
        self.use_truth = use_truth
        # 稳态误差统计的起始时刻。底座先于算法进程启动，接管前那一段
        # 自车是"无人驾驶"直行的，把这部分算进横向误差，量到的是接管
        # 延迟而不是跟踪质量——实验里必须把两者分开报。
        self.warmup_seconds = warmup_seconds
        self.verbose = verbose

        self.perception = Perception()
        self.planner = Planner(planning_cfg)

        self._udp: socket.socket | None = None
        self._tcp: socket.socket | None = None
        self._decoder = FrameDecoder()
        self._clouds = CloudReassembler()
        self._outbuf = bytearray()
        self._latest_cloud: CloudFrame | None = None
        self._last_frame: int | None = None
        self._seq = 0
        self._connected_at = 0.0

        # 统计量。实验章节需要这些数字，因此不是调试打印而是产品输出。
        self.frames = 0
        self.commands = 0
        self.cloud_frames = 0
        self.cloud_points = 0
        self.compute_ms_total = 0.0
        self.min_gap = math.inf
        self.min_ttc = math.inf
        self.max_accel = 0.0
        self.reasons: dict[str, int] = {}
        self.last_plan = None
        self.last_track_count = 0

        # 闭环精度：按帧累加平方和，最后开方得 RMS。不存逐帧样本，
        # 30 s × 50 Hz = 1500 个点虽然不多，但没有保留的必要——
        # 实验要的是 RMS 与峰值，不是波形。
        self.lat_sq_total = 0.0
        self.lat_peak = 0.0
        self.heading_sq_total = 0.0
        self.speed_sq_total = 0.0
        self.error_frames = 0

        # 通信时延：seq → 发出时刻。只保留最近 256 条，避免长跑时无界增长。
        self._send_times: dict[int, float] = {}
        self.rtt_ms: list[float] = []
        # 帧周期实测值（EMA）。净时延要减掉一个周期，而这个周期不该写死 20 ms：
        # 底座改步长或改发布频率时，写死的那份会静默算错。
        self._period_ms = 20.0
        self._prev_sim_time: float | None = None

        # 异常诊断：只在首次出现时各打印一条，避免刷屏。
        # 这两类异常（速度尖峰、零车距）是感知链路最典型的失效表现，
        # 保留它们的输出比事后翻日志猜要省事得多。
        self._spike_logged = 0
        self._near_logged = 0
        self._report_at = 0.0
        self._cloud_age = 0

    # ------------------------------------------------------------ 连接

    def connect(self, wait_seconds: float = 10.0) -> None:
        """绑定点云端口并连接底座。

        先绑 UDP 再连 TCP：底座在 accept 之后就会立刻推送点云，
        如果先连 TCP，最早几帧点云会因为端口还没绑定而丢失。
        """
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        try:
            self._udp.bind((self.host, self.udp_port))
        except OSError as exc:
            raise RuntimeError(
                f"无法绑定点云端口 {self.host}:{self.udp_port}（{exc}）。"
                "多半是已经有一个算法进程在运行。"
            ) from exc
        self._udp.setblocking(False)

        deadline = time.monotonic() + wait_seconds
        last_error: Exception | None = None
        while True:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                sock.connect((self.host, self.tcp_port))
            except OSError as exc:
                last_error = exc
                sock.close()
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f"连接 {self.host}:{self.tcp_port} 失败（{last_error}）。"
                        "请先启动 adasim，或用 --wait 延长等待时间。"
                    ) from exc
                time.sleep(0.2)
                continue
            sock.setblocking(False)
            self._tcp = sock
            break

        self._connected_at = time.monotonic()
        if self.verbose:
            mode = "真值对照" if self.use_truth else "点云感知"
            print(f"已连接 {self.host}:{self.tcp_port}，点云监听 {self.udp_port}，"
                  f"感知来源：{mode}")

    # ------------------------------------------------------------ 主循环

    def run(self, duration: float | None = None) -> dict:
        if self._tcp is None or self._udp is None:
            raise RuntimeError("尚未连接，请先调用 connect()")

        sel = selectors.DefaultSelector()
        sel.register(self._udp, selectors.EVENT_READ, "udp")
        sel.register(self._tcp, selectors.EVENT_READ, "tcp")

        deadline = time.monotonic() + duration if duration else None
        self._report_at = time.monotonic() + 1.0
        closed = False

        try:
            while not closed:
                events = sel.select(0.05)
                # 先排空 UDP 再处理 TCP：底座在同一拍里先写状态后发点云，
                # 但两者到达顺序由内核决定。优先取点云能让本帧的决策
                # 用到同帧观测，而不是上一帧的。
                for key, _ in sorted(events, key=lambda e: 0 if e[0].data == "udp" else 1):
                    if key.data == "udp":
                        self._drain_udp()
                    else:
                        closed = self._drain_tcp()
                        if closed:
                            break

                self._flush()
                now = time.monotonic()
                if self.verbose and now >= self._report_at:
                    self._report(now)
                    self._report_at = now + 1.0
                if deadline is not None and now >= deadline:
                    break
        finally:
            sel.close()
            self.close()

        return self.summary()

    # ------------------------------------------------------------ 收发

    def _drain_udp(self) -> None:
        # 单次排空设上限，而不是"一直读到 EWOULDBLOCK"。
        #
        # 这个上限是压测逼出来的：十万点一帧是 858 个分片，底座按 10 Hz 连发，
        # 接收缓冲里几乎总有下一个分片在路上。若一次事件里把 UDP 读干净，
        # TCP 事件永远轮不到处理——实测 18 s 内状态帧 0 个、指令一条没发，
        # 算法进程实际上没见过任何一帧车辆状态。表现为"点云收到了、车不动"，
        # 极难从现象反推原因。
        #
        # 320 片约覆盖 1/3 帧，足够摊销 select 调用开销，又能保证
        # 每轮循环都给 TCP 留出处理机会。
        for _ in range(320):
            try:
                datagram, _addr = self._udp.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError:
                return

            cloud = self._clouds.feed(datagram)
            if cloud is None:
                continue
            self._latest_cloud = cloud
            self.cloud_frames += 1
            self.cloud_points += len(cloud)

    def _drain_tcp(self) -> bool:
        """返回 True 表示对端已关闭。"""
        while True:
            try:
                chunk = self._tcp.recv(65536)
            except BlockingIOError:
                return False
            except OSError:
                return True
            if not chunk:
                if self.verbose:
                    print("对端已断开连接")
                return True
            for payload in self._decoder.feed(chunk):
                state = parse_state(payload)
                if state is not None:
                    self._on_state(state)

    def _on_state(self, state: StateFrame) -> None:
        if self.frames == 0 and self.verbose:
            # 首个状态帧的仿真时刻就是"接管延迟"：它等于底座从启动到
            # 本进程发出第一条指令之间跑过的仿真时间。这个数字直接决定
            # 首帧工况是否已经跑偏，值得显式打出来而不是让人猜。
            print(f"首个状态帧：frame={state.frame} t={state.sim_time:.2f}s "
                  f"（底座已先行 {state.sim_time:.2f}s）")
        self.frames += 1

        if self._last_frame is not None and state.frame <= self._last_frame:
            # 回溯会让底座重发历史帧号。此时必须重置滤波器状态，
            # 否则航迹会被"预测到未来再被拉回过去"，速度估计直接崩掉。
            self.perception.reset()
            self.planner.reset()
        self._last_frame = state.frame

        if self._prev_sim_time is not None:
            span = state.sim_time - self._prev_sim_time
            if 0.0 < span < 0.5:      # 回溯时 sim_time 会倒退，跳过
                self._period_ms = 0.8 * self._period_ms + 0.2 * span * 1000.0
        self._prev_sim_time = state.sim_time

        begin = time.perf_counter()

        if self.use_truth:
            tracks = _tracks_from_truth(state)
        else:
            tracks = self.perception.process(self._latest_cloud)
            if self._latest_cloud is not None:
                self._cloud_age = state.frame - self._latest_cloud.frame

        dt = 0.02
        plan = self.planner.plan(state.ego, tracks, dt, state.curvature,
                                 state.road_segments, state.odometer)
        self.last_plan = plan
        self.last_track_count = len(tracks)

        # 闭环精度取真值位姿与参考路径的偏差：控制器输出的转角是手段，
        # 车实际偏离参考线多少才是要考核的结果。统计只从 warmup 之后开始。
        if state.sim_time >= self.warmup_seconds:
            self.lat_sq_total += plan.lat_error * plan.lat_error
            self.lat_peak = max(self.lat_peak, plan.lat_error)
            self.heading_sq_total += plan.heading_error * plan.heading_error
            speed_error = state.ego.speed - plan.target_speed
            self.speed_sq_total += speed_error * speed_error
            self.error_frames += 1

        # 时延：状态帧里回带的 ackSeq 对应的就是"底座已消费的最早一条指令"。
        if state.ack_seq >= 0:
            sent = self._send_times.pop(state.ack_seq, None)
            if sent is not None:
                self.rtt_ms.append((time.perf_counter() - sent) * 1000.0)

        # 单帧耗时覆盖"感知 + 决策"整段：begin 打在感知之前，
        # 因此这里的差值已经包含感知，不能再叠加一次感知耗时。
        self.compute_ms_total += (time.perf_counter() - begin) * 1000.0
        self.commands += 1
        self.reasons[plan.reason] = self.reasons.get(plan.reason, 0) + 1

        if plan.lead_id is not None:
            lead = next((t for t in tracks if t.id == plan.lead_id), None)
            if lead is not None:
                # 速度尖峰判据必须与场景无关。早先写死"前车恒为 4.2 m/s，
                # 超过 8 就是失常"，换到切入场景（前车 9 m/s）后每帧都误报——
                # 这类把某个场景的常数当物理事实的断言，比没有诊断更误导人。
                # 改为相对自车车速的上界：滤波器被放大时会出现远超自车量级的速度。
                spike_limit = max(15.0, 2.0 * state.ego.speed + 5.0)
                if lead.speed > spike_limit and self._spike_logged < 3:
                    self._spike_logged += 1
                    print(f"  [速度尖峰] frame={state.frame} v={lead.speed:.2f} "
                          f"限={spike_limit:.1f} vx={lead.vx:+.2f} "
                          f"pos=({lead.x:.2f},{lead.y:.2f}) "
                          f"L={lead.length:.2f} 点云帧="
                          f"{-1 if self._latest_cloud is None else self._latest_cloud.frame}")
                if plan.gap < 3.0 and self._near_logged < 3:
                    self._near_logged += 1
                    print(f"  [近距告警] frame={state.frame} gap={plan.gap:.2f} "
                          f"ego=({state.ego.x:.1f},{state.ego.y:.2f}) "
                          f"lead=({lead.x:.1f},{lead.y:.1f}) v={lead.speed:.2f}")

            self.min_gap = min(self.min_gap, plan.gap)
            self.min_ttc = min(self.min_ttc, plan.ttc)
        self.max_accel = max(self.max_accel, abs(plan.accel))

        self._seq = (self._seq + 1) & 0x7FFFFFFF
        self._send_times[self._seq] = time.perf_counter()
        if len(self._send_times) > 256:
            # 底座没回带的旧序号不会再被匹配，直接丢最旧的。
            for old in list(self._send_times)[:64]:
                self._send_times.pop(old, None)
        self._outbuf += encode_command(
            self._seq, state.frame, state.sim_time,
            plan.steer, plan.target_speed, plan.accel,
            reason=plan.reason,
            # 回传本端实测的往返时延均值，供底座 HUD 显示。底座测不到这个量
            # （它不知道指令在对端被处理了多久），只能由这里带回去。
            # 用均值而不是最近一个样本：逐样本是 20±12 ms 的跳变，
            # HUD 上闪个不停会让人以为网络在抽风，均值才反映链路水平。
            # 注意它天然含一个帧周期（ackSeq 随状态帧回带），净时延见算法汇总。
            rtt_ms=self._mean_rtt_ms(),
        )

    def _mean_rtt_ms(self) -> float | None:
        """往返时延均值，ms。

        这里不扣帧周期：HUD 要显示的是"链路上真实发生的往返"，扣掉之后
        环回场景会常年显示 0.x ms，即使加载到链路打满也看不出变化。
        净时延（扣周期）在汇总里单独给出，两者口径不同，不该混成一个数。
        """
        if not self.rtt_ms:
            return None
        return sum(self.rtt_ms) / len(self.rtt_ms)

    def _flush(self) -> None:
        while self._outbuf:
            try:
                sent = self._tcp.send(self._outbuf)
            except BlockingIOError:
                return
            except OSError:
                return
            if sent <= 0:
                return
            del self._outbuf[:sent]

    # ------------------------------------------------------------ 输出

    def _report(self, now: float) -> None:
        elapsed = now - self._connected_at
        rate = self.frames / elapsed if elapsed > 0.0 else 0.0
        plan = self.last_plan
        pts = (self.cloud_points / self.cloud_frames) if self.cloud_frames else 0.0
        per_frame = (self.compute_ms_total / self.frames) if self.frames else 0.0

        if plan is None:
            print(f"[{elapsed:5.1f}s] 帧 {self.frames:4d} 等待状态报文…")
            return

        gap_text = "  -  " if plan.lead_id is None else f"{plan.gap:5.1f}m"
        ttc_text = "  inf" if math.isinf(plan.ttc) else f"{plan.ttc:5.1f}s"
        print(
            f"[{elapsed:5.1f}s] 帧 {self.frames:4d} {rate:5.1f}Hz | "
            f"点云 {self.cloud_frames:4d}帧 {pts:5.1f}点 龄{self._cloud_age:2d} | "
            f"目标 {self.last_track_count:2d} | "
            f"{plan.reason:9s} gap {gap_text} ttc {ttc_text} | "
            f"δ {plan.steer:+.3f} a {plan.accel:+.2f} | {per_frame:5.2f}ms/帧"
        )

    def summary(self) -> dict:
        elapsed = max(1e-6, time.monotonic() - self._connected_at)
        frames = self.error_frames
        n = max(1, frames)
        rtts = sorted(self.rtt_ms)
        if rtts:
            rtt_mean = sum(rtts) / len(rtts)
            rtt_std = math.sqrt(sum((x - rtt_mean) ** 2 for x in rtts) / len(rtts))
            rtt_p99 = rtts[min(len(rtts) - 1, int(0.99 * (len(rtts) - 1)))]
        else:
            rtt_mean = rtt_std = rtt_p99 = float("nan")
        # 往返时延里必然含一个状态帧周期：指令在下一帧状态才可能被回带。
        # 扣掉它才是"链路 + 底座处理"的净时延，两者分开报，避免把
        # 采样周期算进通信开销里自欺欺人。
        rate = self.frames / elapsed if elapsed > 0.0 else 0.0
        period_ms = 1000.0 / rate if rate > 0.0 else 0.0

        return {
            "运行时长": elapsed,
            "状态帧": self.frames,
            "指令帧": self.commands,
            "点云帧": self.cloud_frames,
            "平均点数": (self.cloud_points / self.cloud_frames) if self.cloud_frames else 0.0,
            "平均计算": (self.compute_ms_total / self.frames) if self.frames else 0.0,
            "最小车距": None if math.isinf(self.min_gap) else self.min_gap,
            "最小TTC": None if math.isinf(self.min_ttc) else self.min_ttc,
            "最大加速度": self.max_accel,
            "决策分布": dict(self.reasons),
            "失步次数": self._decoder.resyncs,
            "丢片帧": self._clouds.dropped,
            "坏包": self._clouds.corrupt,
            "横向误差RMS": math.sqrt(self.lat_sq_total / n) if frames else None,
            "横向误差峰值": self.lat_peak if frames else None,
            "航向误差RMS": math.sqrt(self.heading_sq_total / n) if frames else None,
            "速度误差RMS": math.sqrt(self.speed_sq_total / n) if frames else None,
            "统计帧": frames,
            "时延样本": len(rtts),
            "往返时延均值": rtt_mean,
            "往返时延标准差": rtt_std,
            "往返时延P99": rtt_p99,
            "名义帧周期": period_ms,
            "净时延均值": rtt_mean - period_ms if rtts else float("nan"),
        }

    def close(self) -> None:
        for sock in (self._tcp, self._udp):
            if sock is None:
                continue
            try:
                sock.close()
            except OSError:
                pass
        self._tcp = None
        self._udp = None


def _tracks_from_truth(state: StateFrame) -> list[Track]:
    """对照实验用：把真值障碍物直接当作航迹，跳过感知。

    这条路径存在的意义是量化"感知误差对闭环的影响"：
    同一控制律分别跑点云链路与真值链路，两者指标之差即为感知环节的代价。
    """
    tracks: list[Track] = []
    for o in state.obstacles:
        tracks.append(
            Track(
                id=o.id,
                x=o.x,
                y=o.y,
                theta=o.theta,
                vx=o.speed * math.cos(o.theta),
                vy=o.speed * math.sin(o.theta),
                length=o.length,
                width=o.width,
                hits=99,
            )
        )
    return tracks
