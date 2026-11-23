"""场景生成有效性实验（E5）。

三档口径逐级收紧，分别回答三个不同的问题：

  1. **结构合法率** —— 生成结果是否符合 JSON Schema。这一档最宽，
     只说明"字段没写错"，不代表场景能跑。
  2. **语义合理率** —— 车距为正、触发时刻落在时长内、初始位姿不重叠。
     这一档能挡掉"字段对但物理上讲不通"的场景。
  3. **底座可加载率** —— 把生成结果写盘后交给 C++ 侧 `adasim_batch` 加载
     并跑 5 s。这是唯一能证明"生成的场景真的能跑"的口径；只做前两档，
     很容易出现 Python 校验通过、C++ 解析器不认的字段分歧。

同时统计回退到预置模板的比例与"待人工确认字段"数，这两个数直接对应
"语言驱动的场景构建到底帮人省了多少事"。

用法（项目根目录，需 Qt bin 在 PATH）：
    python tools/eval_scenario_gen.py
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "ai"))

from adasim_ai.generate import generate                      # noqa: E402
from adasim_ai.validate import semantic_ok, structure_ok    # noqa: E402

DATASET = ROOT / "ai" / "datasets" / "nl_scenarios.jsonl"
OUT_DIR = ROOT / "output" / "experiments"
NL_DIR = OUT_DIR / "nl"
BATCH = ROOT / "build" / "qt" / "bin" / "adasim_batch.exe"


def load_dataset() -> list[dict]:
    rows = []
    with open(DATASET, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def batch_check(paths: list[Path]) -> dict[str, dict]:
    """一次起进程、逐行请求，避免 50 次进程启动把耗时算进实验。"""
    if not paths:
        return {}
    if not BATCH.exists():
        return {str(p): {"ok": False, "error": "adasim_batch 未构建"} for p in paths}

    proc = subprocess.Popen([str(BATCH), "--root", str(ROOT)],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, text=True, encoding="utf-8")
    out: dict[str, dict] = {}
    try:
        for path in paths:
            req = json.dumps({"path": str(path), "closedLoop": True,
                              "duration": 5.0, "noise": 0.0, "seed": 1})
            proc.stdin.write(req + "\n")
            proc.stdin.flush()
            line = proc.stdout.readline()
            try:
                out[str(path)] = json.loads(line)
            except json.JSONDecodeError:
                out[str(path)] = {"ok": False, "error": "无法解析批量进程输出"}
    finally:
        proc.stdin.close()
        proc.wait(timeout=30)
    return out


def main() -> int:
    rows = load_dataset()
    NL_DIR.mkdir(parents=True, exist_ok=True)

    results = []
    saved: list[Path] = []
    for item in rows:
        res = generate(item["text"], use_llm=False)
        path = NL_DIR / f"{item['id']}.json"
        saved_ok = False
        try:
            res.scenario.save(str(path))
            saved_ok = True
            saved.append(path)
        except Exception:  # noqa: BLE001 - 保存失败本身就是一条待统计的结果
            saved_ok = False

        results.append({
            "id": item["id"],
            "category": item["category"],
            "text": item["text"],
            "route": res.route,
            "preset": res.preset_name,
            "structure": structure_ok(res.issues),
            "semantic": semantic_ok(res.issues),
            "issues": [str(i) for i in res.issues],
            "unresolved": list(res.scenario.unresolved),
            "actors": len(res.scenario.actors),
            "egoSpeed": res.scenario.ego_speed,
            "duration": res.scenario.duration,
            "saved": saved_ok,
        })

    checks = batch_check([p for p in saved])
    for row, path in zip([r for r in results if r["saved"]], saved):
        info = checks.get(str(path), {})
        row["loadable"] = bool(info.get("ok"))
        if not info.get("ok"):
            row["issues"].extend(info.get("issues") or [info.get("error", "加载失败")])

    # ---- 汇总 ----
    def rate(items: list[dict], key: str) -> float:
        return sum(1 for r in items if r.get(key)) / len(items) if items else 0.0

    by_cat: dict[str, list[dict]] = defaultdict(list)
    for r in results:
        by_cat[r["category"]].append(r)

    summary = {
        "样本数": len(results),
        "结构合法率": rate(results, "structure"),
        "语义合理率": rate(results, "semantic"),
        "底座可加载率": rate(results, "loadable"),
        "回退模板率": sum(1 for r in results if r["route"] == "preset") / len(results),
        "规则解析率": sum(1 for r in results if r["route"] == "rule") / len(results),
        "平均待确认字段": sum(len(r["unresolved"]) for r in results) / len(results),
        "分类": {},
    }
    for cat, items in by_cat.items():
        summary["分类"][cat] = {
            "样本": len(items),
            "结构合法率": rate(items, "structure"),
            "语义合理率": rate(items, "semantic"),
            "底座可加载率": rate(items, "loadable"),
            "回退模板率": sum(1 for r in items if r["route"] == "preset") / len(items),
            "平均待确认字段": sum(len(r["unresolved"]) for r in items) / len(items),
        }

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    with open(OUT_DIR / "summary_e5.json", "w", encoding="utf-8") as fh:
        json.dump({"summary": summary, "detail": results}, fh, ensure_ascii=False, indent=2)

    print(f"样本 {summary['样本数']} 条")
    for key in ("结构合法率", "语义合理率", "底座可加载率", "回退模板率", "规则解析率"):
        print(f"  {key:8s}: {summary[key] * 100:5.1f}%")
    print(f"  平均待确认字段: {summary['平均待确认字段']:.2f}")
    print("\n分类明细：")
    for cat, s in summary["分类"].items():
        print(f"  {cat:5s} n={s['样本']:2d}  结构 {s['结构合法率'] * 100:5.1f}%  "
              f"语义 {s['语义合理率'] * 100:5.1f}%  可加载 {s['底座可加载率'] * 100:5.1f}%  "
              f"回退 {s['回退模板率'] * 100:5.1f}%  待确认 {s['平均待确认字段']:.1f}")
    print(f"\n明细已写出：{OUT_DIR / 'summary_e5.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
