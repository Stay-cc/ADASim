"""场景生成器命令行入口。

用法：
    python ai/scenario_gen.py "前车 25 米同速跟车，第 3 秒邻道车切入并刹车"
    python ai/scenario_gen.py "..." --out scenarios/gen_001.json
    python ai/scenario_gen.py "..." --llm           启用语言模型通路
    python ai/scenario_gen.py --check scenarios/cut_in_brake.json
    python ai/scenario_gen.py --presets             列出并校验预置模板
    python ai/scenario_gen.py --self-test           离线自检

退出码：0 成功；1 生成但校验未通过；2 参数或文件错误。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

# 允许从项目根目录直接运行：`python ai/scenario_gen.py`
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from adasim_ai.generate import PRESET_DIR, generate, load_presets  # noqa: E402
from adasim_ai.nlu import LlmParser  # noqa: E402
from adasim_ai.template import Scenario  # noqa: E402
from adasim_ai.validate import check_all, semantic_ok, structure_ok  # noqa: E402


def cmd_check(path: str) -> int:
    try:
        scenario = Scenario.load(path)
    except (OSError, ValueError) as exc:
        print(f"无法读取 {path}: {exc}")
        return 2

    issues = check_all(scenario)
    if not issues:
        print(f"{path}: 结构与语义均通过")
        return 0

    print(f"{path}: 发现 {len(issues)} 个问题")
    for issue in issues:
        print("  " + str(issue))
    return 0 if structure_ok(issues) and semantic_ok(issues) else 1


def cmd_presets() -> int:
    presets = load_presets()
    if not presets:
        print(f"预置模板目录为空或不存在: {PRESET_DIR}")
        return 2

    bad = 0
    print(f"预置模板（{PRESET_DIR}）：")
    for preset in presets:
        issues = check_all(preset)
        status = "通过" if not issues else f"{len(issues)} 个问题"
        print(f"  {preset.id:28s} {preset.name:14s} {len(preset.actors)} 参与者  {status}")
        for issue in issues:
            print("      " + str(issue))
            bad += 1
    return 0 if bad == 0 else 1


def cmd_generate(args: argparse.Namespace) -> int:
    result = generate(args.text, use_llm=args.llm)

    print(f"场景     : {result.scenario.name}（{result.scenario.id}）")
    print(f"自车     : v={result.scenario.ego_speed:.1f} m/s  "
          f"时长 {result.scenario.duration:.0f} s  "
          f"附着 {result.scenario.friction:.2f}  "
          f"{result.scenario.lane_count} 车道")
    print(f"参与者   : {len(result.scenario.actors)} 个")
    for actor in result.scenario.actors:
        print(f"  - {actor.type} @ x={actor.x:.1f} y={actor.y:.1f} "
              f"v={actor.speed:.1f} 行为={actor.behavior}")
    print(result.report())

    if args.out:
        if not result.ok:
            print("\n校验未通过，未写出文件。加 --force 可强制写出。")
            if not args.force:
                return 1
        result.scenario.save(args.out)
        print(f"\n已写出: {args.out}")

    if args.json:
        print(json.dumps({
            "scenario": result.scenario.to_json(),
            "route": result.route,
            "attempts": result.attempts,
            "ok": result.ok,
            "issues": [i.as_dict() for i in result.issues],
            "unresolved": result.scenario.unresolved,
            "notes": result.notes,
        }, ensure_ascii=False, indent=2))

    return 0 if result.ok else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="自然语言 → ADASim 场景模板",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument("text", nargs="?", help="场景的中文描述")
    parser.add_argument("--out", "-o", help="写出场景文件的路径")
    parser.add_argument("--llm", action="store_true",
                        help="启用语言模型通路（需 ADASIM_LLM_* 环境变量）")
    parser.add_argument("--json", action="store_true", help="以 JSON 打印完整结果")
    parser.add_argument("--force", action="store_true", help="校验未通过也写出")
    parser.add_argument("--check", metavar="FILE", help="只校验一个场景文件")
    parser.add_argument("--presets", action="store_true", help="列出并校验预置模板")
    parser.add_argument("--llm-status", action="store_true", help="查看模型配置状态")
    parser.add_argument("--self-test", action="store_true", help="运行离线自检")

    args = parser.parse_args(argv)

    if args.self_test:
        from selftest import run_all
        return run_all()
    if args.check:
        return cmd_check(args.check)
    if args.presets:
        return cmd_presets()
    if args.llm_status:
        print(f"LLM: {LlmParser().describe()}")
        return 0
    if not args.text:
        parser.print_help()
        return 2
    return cmd_generate(args)


if __name__ == "__main__":
    sys.exit(main())
