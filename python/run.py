"""算法进程入口。

用法：
    python run.py                     连接 127.0.0.1:45100，持续运行
    python run.py --duration 20       跑 20 秒后自动退出并打印汇总
    python run.py --truth             跳过感知，直接用场景真值做对照
    python run.py --self-test         不连仿真器，只跑离线自检
"""

from __future__ import annotations

import argparse
import json
import math
import sys

from adasim_algo.node import AlgoNode
from adasim_algo.perception import Detection, Track, Tracker, cluster_points, fit_box
from adasim_algo.planning import Planner, PlanningConfig, select_lead
from adasim_algo.protocol import (
    CLOUD_HEADER,
    POINT,
    CloudReassembler,
    FrameDecoder,
    encode_command,
    parse_state,
)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="ADASim 算法进程：经 TCP 收状态、经 UDP 收点云、回发控制指令",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--host", default="127.0.0.1", help="底座地址")
    p.add_argument("--tcp-port", type=int, default=45100, help="状态通道端口")
    p.add_argument("--udp-port", type=int, default=45101, help="点云通道端口")
    p.add_argument("--wait", type=float, default=10.0, help="等待底座就绪的秒数")
    p.add_argument("--duration", type=float, default=None, help="运行时长，缺省为一直运行")
    p.add_argument("--cruise", type=float, default=11.0, help="期望车速，m/s")
    p.add_argument("--truth", action="store_true", help="用场景真值替代点云感知（对照实验）")
    p.add_argument("--quiet", action="store_true", help="不打印每秒状态行")
    p.add_argument("--self-test", action="store_true", help="运行离线自检后退出")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    if args.self_test:
        return 0 if self_test() else 1

    cfg = PlanningConfig(cruise_speed=args.cruise)
    node = AlgoNode(
        host=args.host,
        tcp_port=args.tcp_port,
        udp_port=args.udp_port,
        use_truth=args.truth,
        verbose=not args.quiet,
        planning_cfg=cfg,
    )

    try:
        node.connect(wait_seconds=args.wait)
    except RuntimeError as exc:
        print(f"启动失败：{exc}", file=sys.stderr)
        return 2

    try:
        summary = node.run(duration=args.duration)
    except KeyboardInterrupt:
        summary = node.summary()

    _print_summary(summary)
    return 0


def _print_summary(s: dict) -> None:
    print("\n---- 会话汇总 ----")
    print(f"运行时长     : {s['运行时长']:6.1f} s")
    print(f"状态帧/指令帧: {s['状态帧']} / {s['指令帧']}")
    print(f"点云帧/点数  : {s['点云帧']} / 平均 {s['平均点数']:.1f} 点")
    print(f"单帧计算     : {s['平均计算']:.3f} ms")
    gap = "无前车" if math.isinf(s["最小车距"]) else f"{s['最小车距']:.2f} m"
    ttc = "无" if math.isinf(s["最小TTC"]) else f"{s['最小TTC']:.2f} s"
    print(f"最小车距/TTC : {gap} / {ttc}")
    print(f"最大加速度   : {s['最大加速度']:.2f} m/s^2")
    print(f"决策分布     : {s['决策分布']}")
    if s["失步次数"] or s["丢片帧"] or s["坏包"]:
        print(f"协议异常     : 失步 {s['失步次数']} / 丢片帧 {s['丢片帧']} / 坏包 {s['坏包']}")
    else:
        print("协议异常     : 无")


# ---------------------------------------------------------------- 自检


def self_test() -> bool:
    """离线自检：不依赖仿真器，覆盖协议、聚类、跟踪、决策四个环节。

    这些断言的作用不是"证明代码正确"，而是把线上协议里最容易悄悄写错的
    几处（字节序、分片重组、朝向二义性）钉死成可回归的检查。
    """
    checks: list[tuple[str, bool, str]] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        checks.append((name, ok, detail))

    # ---- 1. TCP 指令编码与分帧 ----
    blob = encode_command(7, 132, 2.64, -0.05, 11.0, -0.7)
    dec = FrameDecoder()
    frames = dec.feed(blob)
    check("指令分帧", len(frames) == 1, f"得到 {len(frames)} 帧")
    if frames:
        obj = json.loads(frames[0])
        check("指令字段", obj.get("type") == "cmd" and obj.get("frame") == 132
              and abs(obj.get("accel", 0.0) + 0.7) < 1e-9, str(obj))

    # ---- 2. 半包与粘包 ----
    dec2 = FrameDecoder()
    half = len(blob) // 2
    a = dec2.feed(blob[:half])
    b = dec2.feed(blob[half:] + blob)
    check("半包/粘包", len(a) == 0 and len(b) == 2, f"半包 {len(a)}，续传 {len(b)}")

    # ---- 3. 状态报文解析 ----
    payload = json.dumps({
        "type": "state", "frame": 9, "t": 0.18,
        "ego": {"x": 1.0, "y": -0.2, "theta": 0.01, "v": 7.5},
        "obstacles": [{"id": 3, "kind": "car", "x": 25.0, "y": 0.0,
                       "theta": 0.0, "v": 4.2, "l": 4.5, "w": 1.9}],
    }).encode()
    st = parse_state(payload)
    check("状态解析", st is not None and st.frame == 9 and len(st.obstacles) == 1
          and abs(st.ego.speed - 7.5) < 1e-9, "" if st else "解析失败")

    # ---- 4. 点云分片重组 ----
    xs = [1.0, 2.0, 3.0, 4.0, 5.0]
    ys = [0.0, 0.5, 1.0, 1.5, 2.0]
    chunks = _make_cloud_chunks(42, 0.84, xs, ys, per_chunk=2)
    reasm = CloudReassembler()
    got = None
    for dgram in reversed(chunks):       # 故意乱序，验证重组不依赖到达顺序
        got = reasm.feed(dgram) or got
    check("分片重组", got is not None and len(got) == len(xs)
          and abs(got.xs[4] - 5.0) < 1e-6, f"点数 {0 if got is None else len(got)}")

    # ---- 5. 聚类与框拟合 ----
    bx, by = _rectangle_outline(25.0, 0.0, 4.5, 1.9)
    clusters = cluster_points(bx, by)
    check("点云聚类", len(clusters) == 1, f"得到 {len(clusters)} 个簇")
    if clusters:
        det = fit_box(bx, by, clusters[0])
        ok = (det is not None
              and abs(det.length - 4.5) < 0.6
              and abs(det.width - 1.9) < 0.4
              and abs(det.x - 25.0) < 0.6)
        check("包围盒拟合", ok,
              f"长 {0 if det is None else det.length:.2f} 宽 "
              f"{0 if det is None else det.width:.2f} 中心 "
              f"{0 if det is None else det.x:.2f}")

    # ---- 6. 两车不应被并成一个簇 ----
    cx1, cy1 = _rectangle_outline(10.0, 0.0, 4.5, 1.9)
    cx2, cy2 = _rectangle_outline(10.0, 3.5, 4.5, 1.9)
    merged = cluster_points(cx1 + cx2, cy1 + cy2)
    check("相邻车道不粘连", len(merged) == 2, f"得到 {len(merged)} 个簇")

    # ---- 7. 跟踪器给出速度 ----
    tracker = Tracker()
    tracks = []
    for k in range(6):
        t = 0.02 * k
        tracks = tracker.update([Detection(20.0 + 4.2 * t, 0.0, 0.0, 4.5, 1.9)], t)
    ok = bool(tracks) and abs(tracks[0].speed - 4.2) < 1.2 and tracks[0].confirmed
    check("速度估计", ok, f"v = {tracks[0].speed:.2f} m/s" if tracks else "无航迹")

    # ---- 8. IDM：有前车应减速，无前车应加速 ----
    from adasim_algo.protocol import EgoState

    planner = Planner(PlanningConfig())
    ego = EgoState(0.0, 0.0, 0.0, 8.0)
    lead = Track(id=1, x=18.0, y=0.0, theta=0.0, vx=4.2, vy=0.0,
                 length=4.5, width=1.9, hits=9)
    plan_follow = planner.plan(ego, [lead], 0.02)
    plan_free = planner.plan(ego, [], 0.02)
    check("跟驰减速", plan_follow.reason == "follow" and plan_follow.accel < 0.0,
          f"{plan_follow.reason} a={plan_follow.accel:.2f}")
    check("空域加速", plan_free.reason == "cruise" and plan_free.accel > 0.0,
          f"{plan_free.reason} a={plan_free.accel:.2f}")

    # ---- 9. 走廊判定：邻道车辆不应当作前车 ----
    beside = Track(id=2, x=10.0, y=3.5, theta=0.0, vx=4.0, vy=0.0,
                   length=4.5, width=1.9, hits=9)
    picked, _gap, _v = select_lead(ego, [beside], PlanningConfig())
    check("走廊过滤", picked is None, "邻道被误判为前车" if picked else "")

    # ---- 10. 横向纠偏方向 ----
    planner2 = Planner(PlanningConfig())
    ego_left = EgoState(0.0, 0.8, 0.0, 8.0)   # 偏左，应向右修
    plan_lat = planner2.plan(ego_left, [], 0.02)
    check("纠偏方向", plan_lat.steer < 0.0, f"δ = {plan_lat.steer:+.4f}")

    # ---- 输出 ----
    width = max(len(n) for n, _, _ in checks) + 2
    passed = 0
    for name, ok, detail in checks:
        mark = "通过" if ok else "失败"
        print(f"  [{mark}] {name.ljust(width)} {detail}")
        passed += 1 if ok else 0
    print(f"\n自检结果：{passed}/{len(checks)} 项通过")
    return passed == len(checks)


def _make_cloud_chunks(frame: int, sim_t: float, xs, ys,
                       per_chunk: int = 2000) -> list[bytes]:
    """按 C++ 侧相同的布局构造 UDP 分片，用于验证解析。"""
    total = (len(xs) + per_chunk - 1) // per_chunk
    out = []
    for c in range(total):
        begin = c * per_chunk
        end = min(begin + per_chunk, len(xs))
        count = end - begin
        head = CLOUD_HEADER.pack(
            b"AS", 1, 0x01 if c == total - 1 else 0x00,
            c, total, frame, sim_t, count, 0,
        )
        body = b"".join(POINT.pack(xs[i], ys[i], 0.5) for i in range(begin, end))
        out.append(head + body)
    return out


def _rectangle_outline(cx: float, cy: float, length: float, width: float,
                       spacing: float = 1.0 / 1.5):
    """按 C++ LidarModel 的采样方式生成矩形轮廓点（逆时针）。"""
    hl, hw = length * 0.5, width * 0.5
    corners = [(hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw)]
    xs, ys = [], []
    for k in range(4):
        x0, y0 = corners[k]
        x1, y1 = corners[(k + 1) % 4]
        dx, dy = x1 - x0, y1 - y0
        n = max(1, int(math.hypot(dx, dy) / spacing))
        for i in range(n + 1):
            t = i / n
            xs.append(cx + x0 + dx * t)
            ys.append(cy + y0 + dy * t)
    return xs, ys


if __name__ == "__main__":
    sys.exit(main())
