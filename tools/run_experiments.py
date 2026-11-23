"""闭环实验编排：起底座、起算法进程、收汇总。

为什么用脚本而不是手工敲命令：
  1. 每个场景都要"先起底座、再起算法进程"，顺序错了会丢前几帧，
     手工做十几次必然出现一次时序错误，而错误的那次数据看不出异常；
  2. 汇总要落成 JSON/CSV 才能进报告，手抄数字迟早抄错；
  3. 同一组实验要能原样重跑，脚本本身就是实验记录。

用法（从项目根目录，需先把 Qt 的 bin 目录加入 PATH）：
    python tools/run_experiments.py --group e2
    python tools/run_experiments.py --group e4
    python tools/run_experiments.py --group all
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BASE_EXE = ROOT / "build" / "qt" / "bin" / "adasim.exe"
OUT_DIR = ROOT / "output" / "experiments"

#: 一次运行 = 一个场景 + 一组算法侧开关。同名场景可以跑多次：
#: stress_dense 跑两遍，一遍走点云感知、一遍用真值，用来把"链路时延"
#: 与"算法侧处理耗时"分开——否则十万点下 100 ms 的往返时延会被误读成
#: 通信开销。
RUNS: dict[str, dict] = {
    "latency_probe": {"scenario": "latency_probe"},
    "stress_dense": {"scenario": "stress_dense"},
    "stress_dense_truth": {"scenario": "stress_dense", "truth": True},
    "following_lead": {"scenario": "following_lead"},
    "curve_tracking": {"scenario": "curve_tracking"},
    "lead_emergency_brake": {"scenario": "lead_emergency_brake"},
    "cut_in_simple": {"scenario": "cut_in_simple"},
    "cut_in_brake": {"scenario": "cut_in_brake"},
    "static_obstacle": {"scenario": "static_obstacle"},
}

#: 实验分组 → 运行清单。巡航车速取场景里的 egoSpeed，保证"期望车速"与
#: 场景设计一致；否则跟驰场景会因为巡航设得更高而一直顶着前车跑。
GROUPS: dict[str, list[str]] = {
    "e2": ["latency_probe", "stress_dense", "stress_dense_truth"],
    "e4": ["following_lead", "curve_tracking", "lead_emergency_brake",
           "cut_in_simple", "cut_in_brake", "static_obstacle"],
}

#: 每个场景的 warmup：等自车稳定到工作点再统计误差。弯道与跟驰都需要
#: 一段收敛时间，切入场景则要等切入动作发生之后再算——但这些场景的
#: 切入发生在 3~5 s，统一取 5 s 会把切入过程排除在外，故单独指定。
WARMUP: dict[str, float] = {
    "following_lead": 1.0,
    "curve_tracking": 5.0,
    "lead_emergency_brake": 2.0,
    "cut_in_simple": 2.0,
    "cut_in_brake": 2.5,
    "static_obstacle": 2.0,
    "latency_probe": 3.0,
    "stress_dense": 3.0,
}


def scenario_meta(path: Path) -> dict:
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def run_one(run_id: str) -> dict:
    spec = RUNS[run_id]
    scenario_id = spec["scenario"]
    path = ROOT / "scenarios" / f"{scenario_id}.json"
    meta = scenario_meta(path)
    duration = float(meta.get("durationSeconds", 20.0))
    cruise = float(meta.get("egoSpeed", 11.0))
    warmup = WARMUP.get(scenario_id, 3.0)

    summary_path = OUT_DIR / f"{run_id}.json"
    summary_path.unlink(missing_ok=True)

    # 底座比算法进程多跑 4 s：给连接与截图留余量，也让算法侧能在场景
    # 结束前就收到最后一帧。
    shot_delay_ms = int((duration + 4.0) * 1000)
    base_cmd = [str(BASE_EXE), str(path),
                "--shot", str(OUT_DIR / f"_{run_id}.png"),
                "--shot-delay", str(shot_delay_ms)]
    node_cmd = [sys.executable, str(ROOT / "python" / "run.py"),
                "--duration", f"{duration - 0.5:.2f}",
                "--cruise", f"{cruise:.2f}",
                "--warmup", f"{warmup:.2f}",
                "--quiet",
                "--summary-json", str(summary_path)]
    if spec.get("truth"):
        node_cmd.append("--truth")

    log_base = open(OUT_DIR / f"_{run_id}_base.log", "w", encoding="utf-8")
    base = subprocess.Popen(base_cmd, cwd=ROOT, stdout=log_base, stderr=subprocess.STDOUT)
    try:
        time.sleep(2.0)   # 等底座开始监听，再让算法进程连
        if base.poll() is not None:
            raise RuntimeError(f"底座提前退出，见 _{{scenario_id}}_base.log")
        node = subprocess.run(node_cmd, cwd=ROOT, capture_output=True, text=True,
                              timeout=duration + 60)
    finally:
        try:
            base.wait(timeout=30)
        except subprocess.TimeoutExpired:
            base.kill()
        log_base.close()

    if node.returncode != 0:
        raise RuntimeError(f"算法进程退出码 {node.returncode}：{node.stderr[-800:]}")

    with open(summary_path, encoding="utf-8") as fh:
        summary = json.load(fh)
    summary["场景"] = scenario_id
    summary["运行"] = run_id
    summary["感知方式"] = "真值" if spec.get("truth") else "点云"
    summary["场景名称"] = meta.get("name", scenario_id)
    summary["期望车速"] = cruise
    summary["warmup"] = warmup
    summary["底板退出码"] = base.returncode
    return summary


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="ADASim 闭环实验编排")
    ap.add_argument("--group", default="all", choices=[*GROUPS, "all"])
    ap.add_argument("--only", default=None, help="只跑某个场景（调试用）")
    args = ap.parse_args(argv)

    if not BASE_EXE.exists():
        print(f"找不到底座：{BASE_EXE}", file=sys.stderr)
        return 2

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    ids = [args.only] if args.only else (
        [s for g in GROUPS.values() for s in g] if args.group == "all" else GROUPS[args.group])

    rows = []
    failures = 0
    for sid in ids:
        print(f"[{sid}] 运行中…", flush=True)
        try:
            row = run_one(sid)
        except Exception as exc:  # noqa: BLE001 - 编排脚本要一份失败也要继续跑下一组
            print(f"[{sid}] 失败：{exc}", file=sys.stderr)
            failures += 1
            continue
        rows.append(row)
        gap = row["最小车距"]
        lat = row["横向误差RMS"]
        spd = row["速度误差RMS"]
        print(f"[{sid}] 帧 {row['状态帧']}，最小净距 "
              f"{'无' if gap is None else format(gap, '.2f') + ' m'}，"
              f"横向 RMS {'-' if lat is None else format(lat, '.3f') + ' m'}，"
              f"速度误差 RMS {'-' if spd is None else format(spd, '.2f') + ' m/s'}",
              flush=True)

    if not rows:
        print("没有产出任何结果", file=sys.stderr)
        return 1

    merged = OUT_DIR / f"summary_{args.group}.json"
    with open(merged, "w", encoding="utf-8") as fh:
        json.dump(rows, fh, ensure_ascii=False, indent=2)
    print(f"\n汇总已写出：{merged}")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
