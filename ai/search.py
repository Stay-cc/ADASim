"""贝叶斯失效搜索命令行入口。

用法：
    python ai/search.py --scenario ai/presets/cut_in_brake.json --budget 40
    python ai/search.py --scenario ... --budget 100 --out ai/out/search.json
    python ai/search.py --self-test

搜索变量取自场景 `parameters` 块中已知绑定的那几个（见 template.apply_parameters），
区间也来自那里。模板没声明的变量不会被搜索——凭空给一个区间会让优化器
去调一个场景根本不支持的维度，白烧预算。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from adasim_ai.search import (  # noqa: E402
    DEFAULT_EXE, BatchEvaluator, BayesianSearch, RandomSearch, severity,
)
from adasim_ai.template import Scenario  # noqa: E402


def build_report(result, scenario: Scenario) -> str:
    """生成 Markdown 结果报告。论文里要的表格基本都能直接用。"""
    best = result.best
    baseline = result.baseline
    lines = [
        f"# 失效搜索报告：{scenario.name}",
        "",
        f"- 场景文件：`{scenario.id}`",
        f"- 评估预算：{result.budget} 次，实际评估 {len(result.trials)} 次",
        f"- 搜索变量：{'、'.join(result.names)}",
        f"- 碰撞样本：{result.collided_count} / {len(result.trials)}",
        "",
        "## 结果对比",
        "",
        "| 参数 | 基线（模板原值） | 搜索最优 | 区间 |",
        "|---|---:|---:|---|",
    ]
    for name in result.names:
        low, high = result.bounds[name]
        base_value = f"{baseline.params[name]:.3g}" if baseline else "—"
        best_value = f"{best.params[name]:.3g}" if best else "—"
        lines.append(f"| {name} | {base_value} | {best_value} | [{low:.3g}, {high:.3g}] |")

    lines += [
        "",
        "| 指标 | 基线 | 搜索最优 |",
        "|---|---:|---:|",
    ]
    if baseline and best:
        lines += [
            f"| 失效严重度 | {baseline.score:.4f} | **{best.score:.4f}** |",
            f"| 是否碰撞 | {'是' if baseline.metrics.collided else '否'} "
            f"| {'是' if best.metrics.collided else '否'} |",
            f"| 最小 TTC (s) | {baseline.metrics.min_ttc:.3f} | {best.metrics.min_ttc:.3f} |",
            f"| 最小净距 (m) | {baseline.metrics.min_gap:.3f} | {best.metrics.min_gap:.3f} |",
            f"| 撞击速度 (m/s) | {baseline.metrics.impact_speed:.3f} "
            f"| {best.metrics.impact_speed:.3f} |",
            f"| 峰值加速度 (m/s²) | {baseline.metrics.max_abs_accel:.3f} "
            f"| {best.metrics.max_abs_accel:.3f} |",
        ]

    risky = sorted((t for t in result.trials if t.phase != "baseline"),
                   key=lambda t: t.score, reverse=True)[:5]
    lines += ["", "## 最危险的五组参数", "",
              "| 严重度 | 碰撞 | 最小 TTC | 最小净距 | " +
              " | ".join(result.names) + " |",
              "|---:|---|---:|---:|" + "---:|" * len(result.names)]
    for trial in risky:
        values = " | ".join(f"{trial.params[n]:.3g}" for n in result.names)
        lines.append(
            f"| {trial.score:.4f} | {'是' if trial.metrics.collided else '否'} "
            f"| {trial.metrics.min_ttc:.2f} | {trial.metrics.min_gap:.2f} | {values} |")

    total_wall = sum(t.metrics.wall_ms for t in result.trials)
    lines += [
        "",
        "## 评估开销",
        "",
        f"- 单次仿真：{total_wall / max(1, len(result.trials)):.2f} ms（均值）",
        f"- 全程仿真耗时：{total_wall:.0f} ms",
        "",
        "> 搜索只在离线批量模式下运行，不进入实时链路；实时帧率不受影响。",
    ]
    return "\n".join(lines)


def run_search(method: str, scenario: Scenario, bounds: dict[str, tuple[float, float]],
               baseline: dict[str, float], args: argparse.Namespace):
    """跑一次搜索。每次新建评估器，保证两种方法在同等条件下比较。"""
    with BatchEvaluator(exe=args.exe, scenario=scenario,
                        noise_seed=args.noise_seed) as evaluator:
        if method == "random":
            searcher = RandomSearch(evaluator, bounds, budget=args.budget, seed=args.seed)
        else:
            searcher = BayesianSearch(evaluator, bounds, budget=args.budget,
                                      init_points=args.init, seed=args.seed)
        return searcher.run(baseline), evaluator.evaluations


def print_summary(result, label: str = "") -> None:
    best = result.best
    prefix = f"{label} " if label else ""
    if result.baseline and best:
        print(f"{prefix}基线严重度 : {result.baseline.score:.4f}"
              f"（碰撞={result.baseline.metrics.collided}，"
              f"TTC={result.baseline.metrics.min_ttc:.2f}s）")
        print(f"{prefix}最优严重度 : {best.score:.4f}"
              f"（碰撞={best.metrics.collided}，"
              f"撞击速度={best.metrics.impact_speed:.2f} m/s）")
        print(f"{prefix}最优参数   : " + "、".join(f"{k}={v:.3g}" for k, v in best.params.items()))
    else:
        print(f"{prefix}未得到有效结果")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="在场景参数空间内搜索最易失效的工况",
        epilog=__doc__)
    parser.add_argument("--scenario", default=os.path.join(
        "ai", "presets", "cut_in_brake.json"), help="场景模板路径")
    parser.add_argument("--budget", type=int, default=40, help="评估预算（默认 40）")
    parser.add_argument("--init", type=int, default=8, help="拉丁超立方初始点数")
    parser.add_argument("--seed", type=int, default=0, help="随机种子")
    parser.add_argument("--noise-seed", type=int, default=1, help="感知噪声的随机种子")
    parser.add_argument("--exe", default=DEFAULT_EXE, help="评估进程路径")
    parser.add_argument("--out", help="把完整结果写成 JSON")
    parser.add_argument("--report", help="把结果写成 Markdown 报告")
    parser.add_argument("--method", choices=("bo", "random", "both"), default="bo",
                        help="bo=贝叶斯（默认）、random=随机基线、both=同预算对比")
    parser.add_argument("--self-test", action="store_true", help="运行搜索模块自检")

    args = parser.parse_args(argv)

    if args.self_test:
        from selftest import run_all
        return run_all()

    if not os.path.isfile(args.scenario):
        print(f"找不到场景文件: {args.scenario}")
        return 2

    scenario = Scenario.load(args.scenario)
    bounds = scenario.search_bounds()
    if not bounds:
        print(f"场景 {scenario.id} 的 parameters 里没有可搜索的变量。")
        print("可用变量名：" + "、".join(
            ("ego_speed", "lead_gap", "cut_in_trigger", "friction", "perception_noise")))
        return 2

    print(f"场景     : {scenario.name}（{scenario.id}）")
    print(f"搜索变量 : {len(bounds)} 个 / 预算 {args.budget} 次")
    for name, (low, high) in bounds.items():
        print(f"  - {name:18s} [{low:g}, {high:g}]")

    # 基线 = 模板原值（参数取当前模板里的实际值，不是搜索下界）
    baseline = {
        "ego_speed": scenario.ego_speed,
        "friction": scenario.friction,
        "perception_noise": 0.0,
    }
    if "lead_gap" in bounds:
        lead = scenario._first_actor_of("car_following") or scenario._first_actor_of("brake")
        baseline["lead_gap"] = lead.x if lead else bounds["lead_gap"][0]
    if "cut_in_trigger" in bounds:
        cutter = scenario._first_actor_of("cut_in")
        baseline["cut_in_trigger"] = cutter.trigger if cutter else bounds["cut_in_trigger"][0]
    baseline = {k: v for k, v in baseline.items() if k in bounds}

    try:
        if args.method == "both":
            print("\n--- 贝叶斯优化 ---")
            bo_result, bo_evals = run_search("bo", scenario, bounds, baseline, args)
            print_summary(bo_result, "BO")
            print("\n--- 随机搜索（同预算）---")
            rnd_result, rnd_evals = run_search("random", scenario, bounds, baseline, args)
            print_summary(rnd_result, "随机")

            bo_best = bo_result.best.score if bo_result.best else 0.0
            rnd_best = rnd_result.best.score if rnd_result.best else 0.0
            bo_hits = sum(1 for t in bo_result.trials if t.metrics.collided and t.phase != "baseline")
            rnd_hits = sum(1 for t in rnd_result.trials if t.metrics.collided and t.phase != "baseline")
            print("\n--- 对比 ---")
            print(f"{'方法':8s} {'最优严重度':>10s} {'碰撞样本':>10s} {'评估次数':>10s}")
            print(f"{'贝叶斯':8s} {bo_best:10.4f} {bo_hits:10d} {bo_evals:10d}")
            print(f"{'随机':8s} {rnd_best:10.4f} {rnd_hits:10d} {rnd_evals:10d}")
            delta = bo_best - rnd_best
            print(f"差值     : {delta:+.4f}"
                  + ("（贝叶斯更优）" if delta > 0 else "（本次随机更优）"))
            result, evaluations = bo_result, bo_evals
        else:
            result, evaluations = run_search(args.method, scenario, bounds, baseline, args)
            print(f"\n评估完成：{len(result.trials)} 次（含基线），"
                  f"实际调用仿真 {evaluations} 次")
            print_summary(result)
            curve = result.convergence()
            if len(curve) > 1:
                print(f"收敛       : 首次 {curve[0]:.4f} → 末次 {curve[-1]:.4f}")
    except (FileNotFoundError, RuntimeError) as exc:
        print(f"\n搜索失败：{exc}")
        return 2

    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as fh:
            json.dump(result.to_json(), fh, ensure_ascii=False, indent=2)
        print(f"结果已写出 : {args.out}")

    if args.report:
        os.makedirs(os.path.dirname(os.path.abspath(args.report)), exist_ok=True)
        with open(args.report, "w", encoding="utf-8") as fh:
            fh.write(build_report(result, scenario) + "\n")
        print(f"报告已写出 : {args.report}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
