#!/usr/bin/env python3
"""生成点云压测用的密集场景（JSON）。

为什么要有这个脚本，而不是手写一份 JSON：

1. 十万点需要七八百个交通参与者，手写不可维护，改一次布局就得改几百行；
2. 点云规模由"车数 × 每车轮廓点数"决定，而每车点数又由采样密度与车身
   尺寸决定（轮廓采样会向下取整），只有算出来才知道到底多少点。本脚本
   复刻 LidarModel 的采样公式给出估算值，避免"以为是十万点、实际只有
   三万"这类低级错误；
3. 压测结论要可复现，参数就必须显式化——所以脚本把布局参数做成命令行
   选项，产出的 JSON 里也带上 sensor 块。

注意：这个场景是**合成载荷**，不是物理上合理的交通流。真实道路上自车
50 m 量程内不会有几百辆车，十万点量级本身就无法由正常车流产生
（详见 docs/性能压测报告.md）。它的用途是把渲染与通信压到设计上限。

用法：
    python tools/gen_dense_scenario.py                       # 默认约十万点
    python tools/gen_dense_scenario.py --lanes 12 --columns 24
"""

from __future__ import annotations

import argparse
import json
import math
import os

CAR_LENGTH = 4.5
CAR_WIDTH = 1.9


def points_per_car(density: float, length: float = CAR_LENGTH, width: float = CAR_WIDTH) -> int:
    """复刻 LidarModel 的轮廓采样点数。

    每条边取 int(边长/间距) 段，再画 samples+1 个点，所以短边会被向上取整
    放大点数：1.9 m 的车宽在 10 点/米下分成 19 段、20 个点。
    """
    spacing = 1.0 / density
    long_pts = int(length / spacing) + 1
    short_pts = int(width / spacing) + 1
    return int(2 * long_pts + 2 * short_pts)


def build(args: argparse.Namespace) -> dict:
    actors = []
    # 车道横坐标：跳过 y=0——那是自车所在车道，留空让自车能往前开，
    # 否则起步即被自己车道上的车堵死，画面变成"停车场"而不是"车流"。
    half_lanes = args.lanes // 2
    lane_ys = []
    for i in range(1, half_lanes + 1):
        lane_ys.append(round(i * args.lane_width, 3))
        lane_ys.append(round(-i * args.lane_width, 3))
    lane_ys.sort()

    for col in range(args.columns):
        x = round(args.start_x + col * args.spacing, 3)
        for y in lane_ys:
            actors.append(
                {
                    "type": "car",
                    "x": x,
                    "y": y,
                    "theta": 0.0,
                    "speed": args.speed,
                    "behavior": "constant",
                    "trigger": 0.0,
                }
            )

    return {
        "id": args.scenario_id,
        "name": args.name,
        "laneCount": args.lanes,
        "laneWidth": args.lane_width,
        "curvature": 0.0,
        "friction": 0.9,
        "visibility": 1.0,
        "durationSeconds": args.duration,
        "sensor": {
            "lidarRange": args.range,
            "lidarPointsPerMeter": args.density,
        },
        "actors": actors,
        "parameters": [
            {"name": "ego_speed", "min": args.speed, "max": args.speed + 4.0, "step": 1.0}
        ],
    }


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="scenarios/stress_dense.json")
    p.add_argument("--scenario-id", default="stress_dense")
    p.add_argument("--name", default="点云压测：密排车阵（合成载荷）")
    p.add_argument("--lanes", type=int, default=24, help="车道总数（含自车所在空车道）")
    p.add_argument("--columns", type=int, default=32, help="纵向排数")
    p.add_argument("--lane-width", type=float, default=3.5)
    p.add_argument("--spacing", type=float, default=5.4, help="纵向车距（车头到车头），m")
    p.add_argument("--start-x", type=float, default=12.0, help="首排车头位置，m")
    p.add_argument("--speed", type=float, default=8.0, help="全体车速，m/s（与自车同速=相对静止）")
    p.add_argument("--density", type=float, default=10.0, help="点云采样密度，点/米（10 即 0.1 m 间距）")
    p.add_argument("--range", type=float, default=250.0, help="激光雷达量程，m")
    p.add_argument("--duration", type=float, default=30.0)
    args = p.parse_args()

    doc = build(args)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(doc, fh, ensure_ascii=False, indent=1)
        fh.write("\n")

    n_actors = len(doc["actors"])
    per_car = points_per_car(args.density)
    far = max(math.hypot(a["x"], a["y"]) for a in doc["actors"])
    in_range = sum(1 for a in doc["actors"] if math.hypot(a["x"], a["y"]) <= args.range)
    print(f"已写出 {args.out}")
    print(f"  参与者        : {n_actors}（量程内 {in_range}）")
    print(f"  每车轮廓点数  : {per_car}（{args.density} 点/米）")
    print(f"  估算总点数    : {in_range * per_car:,}")
    print(f"  最远距离      : {far:.1f} m（量程 {args.range:.0f} m）")
    print(f"  文件大小      : {os.path.getsize(args.out) / 1024:.0f} KB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
