#!/usr/bin/env python3
"""感知侧点云规模压测：十万点进来，纯 Python 扛不扛得住。

为什么单独测这一侧：C++ 侧的生成与渲染都能在毫秒级收工（见
docs/性能压测报告.md），但算法进程是纯 Python、逐点循环。闭环能不能
跑满 50 Hz，瓶颈大概率不在渲染而在感知——这件事必须用数字说清楚，
否则"平台支持十万点"就只是把点画出来了，算法根本没处理。

测量口径与线上一致：直接调 Perception.process()，它内部会按 MAX_POINTS
抽稀，所以测出来的既有抽稀代价也有抽稀收益。用仿真时间而非墙钟推进
航迹，重复 3 次取中位数。

用法：
    python python/bench_perception.py
    python python/bench_perception.py --points 2000 20000 100000 --repeat 5
"""

from __future__ import annotations

import argparse
import math
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from adasim_algo.perception import MAX_POINTS, Perception  # noqa: E402
from adasim_algo.protocol import CloudFrame  # noqa: E402

CAR_LENGTH = 4.5
CAR_WIDTH = 1.9
LANE_WIDTH = 3.5
COL_SPACING = 5.4
LANES = 24


def synth_cloud(total_points: int, points_per_car: int) -> CloudFrame:
    """按密排车阵的几何合成点云：每个目标一圈矩形轮廓点。

    与 C++ 侧 LidarModel 的采样方式一致（沿四条边等距取点），这样
    聚类与框拟合面对的分布和真实链路相同——若改成随机撒点，会低估
    聚类成本（随机点几乎不连通，BFS 邻居表为空）。
    """
    cars = max(1, total_points // points_per_car)
    xs: list[float] = []
    ys: list[float] = []
    inten: list[float] = []

    columns = max(1, cars // LANES)
    for col in range(columns):
        cx = 12.0 + col * COL_SPACING
        for i in range(1, LANES // 2 + 1):
            for sign in (-1.0, 1.0):
                cy = sign * i * LANE_WIDTH
                # 沿周长均匀取 points_per_car 个点（角点在边上重复，无碍）
                for k in range(points_per_car):
                    u = k / points_per_car
                    if u < 0.3515:            # 长边 4.5 / 周长 12.8
                        lx, ly = -CAR_LENGTH / 2 + u / 0.3515 * CAR_LENGTH, CAR_WIDTH / 2
                    elif u < 0.5:
                        lx, ly = CAR_LENGTH / 2, CAR_WIDTH / 2 - (u - 0.3515) / 0.1485 * CAR_WIDTH
                    elif u < 0.8515:
                        lx, ly = CAR_LENGTH / 2 - (u - 0.5) / 0.3515 * CAR_LENGTH, -CAR_WIDTH / 2
                    else:
                        lx, ly = -CAR_LENGTH / 2, -CAR_WIDTH / 2 + (u - 0.8515) / 0.1485 * CAR_WIDTH
                    xs.append(cx + lx)
                    ys.append(cy + ly)
                    inten.append(0.6)

    return CloudFrame(frame=0, sim_time=0.0, xs=tuple(xs), ys=tuple(ys), intensity=tuple(inten))


def run_case(points: int, points_per_car: int, repeat: int) -> None:
    cloud = synth_cloud(points, points_per_car)
    actual = len(cloud.xs)

    timings: list[float] = []
    stride = 0
    clusters = 0
    tracks = 0
    for r in range(repeat):
        perc = Perception()          # 每次都从空航迹开始，避免累积影响
        probe = CloudFrame(frame=r, sim_time=r * 0.02, xs=cloud.xs, ys=cloud.ys,
                           intensity=cloud.intensity)
        t0 = time.perf_counter()
        result = perc.process(probe)
        timings.append((time.perf_counter() - t0) * 1000.0)
        stride = perc.stats.decimated
        clusters = perc.stats.clusters
        tracks = len(result)

    med = statistics.median(timings)
    budget = med / 20.0 * 100.0   # 20 ms 节拍的占用率
    decimated = f"1/{stride}（-> {actual // max(1, stride):,} 点）" if stride else "不抽稀"
    print(f"输入 {actual:>7,} 点  抽稀 {decimated:<24} 簇 {clusters:>4}  航迹 {tracks:>3}  "
          f"耗时 中位 {med:7.1f} ms  最差 {max(timings):7.1f} ms  占 20 ms 节拍 {budget:5.0f}%")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                               formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--points", type=int, nargs="+",
                   default=[2000, 20000, 50000, 100000])
    p.add_argument("--per-car", type=int, default=130, help="每车轮廓点数（0.1 m 采样即 130）")
    p.add_argument("--repeat", type=int, default=3)
    args = p.parse_args()

    print(f"感知压测：MAX_POINTS = {MAX_POINTS}，每车 {args.per_car} 点，重复 {args.repeat} 次取中位\n")
    for n in args.points:
        run_case(n, args.per_car, args.repeat)

    print("\n注：抽稀是按索引等步长取样，会打散轮廓的连续性，框拟合精度下降；")
    print("    这里只测耗时上限，精度影响见 docs/性能压测报告.md 的说明。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
