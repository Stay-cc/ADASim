"""一条命令跑通整个 Demo：跟驰 → 切入 → 断链，三段连播。

为什么要串成三段
----------------
单场景的 Demo 只能说明"某一种情况能跑"。答辩时评委真正想看的是三件事：
常规跟驰稳不稳、突然有人插进来接不接得住、算法进程掉线了车会不会失控。
这三件事没法在同一个场景里同时演，只能各跑一段再接起来。手工做要起停底座三次、
掐两次时序，错一次看到的就是"算法没连上"，而画面看起来一切正常。

最后一段刻意复用第二段的场景：**同一个切入场景，唯一变量是算法进程死没死**。
两段并排看，断链前后的差别才是可控实验，而不是"换了个场景恰好没撞"。

用法
----
    python tools/demo.py                       # 三段连播，约 1 分钟
    python tools/demo.py --frames 12 --interval 600
    python tools/demo.py --drop-at 5           # 第三段第 5 秒断链
    python tools/demo.py --scenario scenarios/curve_tracking.json   # 退回单场景
    python tools/demo.py --only drop           # 只跑某一段

产物（默认 output/demo/）
    segments/<id>/frames/frame_XX.png   各段连续截图
    segments/<id>/timeseries.csv        100 ms 采样的时间/车速/车距/链路
    segments/<id>/algo.json             算法进程汇总
    index.html                          三段时间轴 + 连播播放器，双击即开
"""

from __future__ import annotations

import argparse
import base64
import csv
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT_DEFAULT = ROOT / "output" / "demo"


# ---------------------------------------------------------------------------
# 三段剧本
# ---------------------------------------------------------------------------

#: online 为 None 表示算法进程全程在线；数字表示在线到第几秒后退出（断链）。
SEGMENTS: list[dict] = [
    {
        "id": "follow",
        "label": "① 跟驰",
        "scenario": "scenarios/following_lead.json",
        "online": None,
        "note": "算法在线。前车 4.2 m/s 匀速，自车 8 m/s 追近后由 IDM 收敛到稳定间距，"
                "不再持续贴近。",
    },
    {
        "id": "cutin",
        "label": "② 切入",
        "scenario": "scenarios/cut_in_brake.json",
        "online": None,
        "note": "算法在线。邻道车在 2.5 s 起横移到自车车道，随后以 2.5 m/s² 减速，"
                "自车让速保距。",
    },
    {
        "id": "drop",
        "label": "③ 断链",
        "scenario": "scenarios/cut_in_brake.json",
        "online": 5.0,
        "note": "与第二段完全相同的场景，唯一变量是算法进程在接入 5 秒后退出。"
                "底座兜底律接管后自行减速，车距不再收缩。",
    },
]


# ---------------------------------------------------------------------------
# 环境准备
# ---------------------------------------------------------------------------

def find_base() -> Path | None:
    """按"先看当前构建目录、再看别的构建目录"的顺序找底座可执行文件。"""
    candidates = [
        ROOT / "build" / "qt" / "bin" / "adasim.exe",
        ROOT / "build" / "bin" / "adasim.exe",
        ROOT / "build" / "qt" / "bin" / "adasim",
        ROOT / "build" / "bin" / "adasim",
    ]
    for c in candidates:
        if c.exists():
            return c
    return None


def check_qt_runtime(base: Path) -> str | None:
    """底座链接 Qt6，缺运行库时进程会在启动瞬间退出且没有可读报错。

    与其让人对着"闪一下就没了"猜半天，不如在这里提前说清楚。
    先尝试从 CMake 缓存里把 Qt 的 bin 与配套 MinGW 的 bin 补进 PATH——
    构建时用的是哪套 Qt，运行时就得用哪套，缓存里记得一清二楚，没道理让人再手填一遍。
    """
    if os.name != "nt":
        return None

    def has_qt_bin() -> bool:
        for p in os.environ.get("PATH", "").split(os.pathsep):
            try:
                if (Path(p) / "Qt6Core.dll").exists():
                    return True
            except OSError:
                continue
        return False

    if has_qt_bin():
        return None

    # 从 build/qt 的缓存推 Qt 安装前缀：.../mingw_64/lib/cmake/Qt6 → .../mingw_64
    cache = ROOT / "build" / "qt" / "CMakeCache.txt"
    if cache.exists():
        for line in cache.read_text(encoding="utf-8", errors="replace").splitlines():
            if line.startswith("Qt6_DIR:PATH="):
                qt6 = Path(line.split("=", 1)[1].strip())
                prefix = qt6.parents[2] if len(qt6.parents) >= 3 else None
                if prefix and (prefix / "bin" / "Qt6Core.dll").exists():
                    extra = [prefix / "bin"]
                    # 同一套 Qt 的 MinGW 工具链（libstdc++/libgcc/libwinpthread）
                    tools = prefix.parent / "Tools"
                    if tools.exists():
                        extra += sorted(tools.glob("mingw*_64/bin"))
                    os.environ["PATH"] = os.pathsep.join(
                        [str(p) for p in extra] + [os.environ.get("PATH", "")])
                    if has_qt_bin():
                        return None
                break

    return (f"没在 PATH 里找到 Qt6Core.dll。请把 Qt 的 bin 目录加入 PATH，例如：\n"
            f"    export PATH=\"/path/to/Qt/6.x/mingw_64/bin:/path/to/Qt/Tools/mingw1310_64/bin:$PATH\"\n"
            f"底座路径: {base}")


# ---------------------------------------------------------------------------
# 跑一段
# ---------------------------------------------------------------------------

def run_segment(spec: dict, out: Path, base: Path, args) -> dict | None:
    """起底座录帧，期中起算法进程，按 spec 决定它活到第几秒。"""
    sid = spec["id"]
    seg_dir = out / "segments" / sid
    frames_dir = seg_dir / "frames"
    if seg_dir.exists():
        shutil.rmtree(seg_dir)      # 上一轮的帧会混进来，看起来像"多录了两帧"
    frames_dir.mkdir(parents=True)

    scenario = ROOT / spec["scenario"]
    meta = json.loads(scenario.read_text(encoding="utf-8"))
    cruise = spec.get("cruise") or meta.get("egoSpeed", 10.0)

    frames = spec.get("frames", args.frames)
    interval = spec.get("interval", args.interval)
    total = frames * interval / 1000.0

    if spec["online"] is None:
        online = max(1.0, total - 0.8)   # 留出收尾，别让算法进程和底座同时退出
    else:
        online = min(spec["online"], total - 2.0)   # 至少留 2 s 给断链后的镜头

    ts_path = seg_dir / "timeseries.csv"
    algo_path = seg_dir / "algo.json"
    log_base = seg_dir / "base.log"

    print(f"[{spec['label']}] {meta.get('name', scenario.stem)}  "
          f"{frames} 帧 × {interval} ms ≈ {total:.1f} s，"
          f"算法在线 {online:.1f} s" + ("（全程）" if spec["online"] is None else "，之后断链"))

    base_cmd = [
        str(base), str(scenario),
        "--record", str(frames_dir),
        "--frames", str(frames),
        "--interval", str(interval),
        "--metrics", str(ts_path),
    ]
    with open(log_base, "w", encoding="utf-8") as log:
        proc = subprocess.Popen(base_cmd, stdout=log, stderr=subprocess.STDOUT)
        try:
            # 底座要先把网关监听起来，算法进程才有地方连。
            time.sleep(args.startup)
            if proc.poll() is not None:
                print("底座启动即退出，看 base.log：\n"
                      + log_base.read_text(encoding="utf-8"), file=sys.stderr)
                return None

            node_cmd = [
                sys.executable, str(ROOT / "python" / "run.py"),
                "--duration", f"{online:.1f}",
                "--cruise", f"{cruise:.1f}",
                "--quiet",
                "--summary-json", str(algo_path),
            ]
            node = subprocess.run(node_cmd, cwd=ROOT, capture_output=True, text=True,
                                  encoding="utf-8", errors="replace")
            if node.returncode != 0 and node.stderr.strip():
                print(node.stderr.strip()[-400:], file=sys.stderr)

            # 等底座录完自己的帧后自行退出。
            proc.wait(timeout=total + 20)
        except subprocess.TimeoutExpired:
            print("底座未按时退出，强制结束", file=sys.stderr)
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        except KeyboardInterrupt:
            proc.terminate()
            raise

    frame_files = sorted(frames_dir.glob("frame_*.png"))
    if not frame_files:
        print("没有拿到任何帧，检查 base.log：\n"
              + log_base.read_text(encoding="utf-8"), file=sys.stderr)
        return None

    series = read_series(ts_path)
    algo = json.loads(algo_path.read_text(encoding="utf-8")) if algo_path.exists() else {}

    return {
        "id": sid,
        "label": spec["label"],
        "title": meta.get("name", scenario.stem),
        "scene": spec["scenario"],
        "note": spec["note"],
        "cruise": cruise,
        "interval": interval,
        "frames": [str(p) for p in frame_files],
        "series": series,
        "algo": algo,
    }


# ---------------------------------------------------------------------------
# 取数与结论
# ---------------------------------------------------------------------------

def load_segment(spec: dict, out: Path) -> dict | None:
    """不重跑仿真，直接把上一轮的产物读回来。

    页面版式改动频繁，而一次三段连播要跑一分钟。改版面就重跑一遍仿真没有意义，
    数据又不会变——留这条路，改前端的时候 `--rebuild` 一秒钟出新页面。
    """
    seg_dir = out / "segments" / spec["id"]
    frames = sorted((seg_dir / "frames").glob("frame_*.png"))
    if not frames:
        return None
    scenario = ROOT / spec["scenario"]
    meta = json.loads(scenario.read_text(encoding="utf-8"))
    algo_path = seg_dir / "algo.json"
    return {
        "id": spec["id"],
        "label": spec["label"],
        "title": meta.get("name", scenario.stem),
        "scene": spec["scenario"],
        "note": spec["note"],
        "cruise": spec.get("cruise") or meta.get("egoSpeed", 10.0),
        "interval": spec.get("interval", 700),
        "frames": [str(p) for p in frames],
        "series": read_series(seg_dir / "timeseries.csv"),
        "algo": json.loads(algo_path.read_text(encoding="utf-8"))
                if algo_path.exists() else {},
    }


def read_series(path: Path) -> list[dict]:
    if not path.exists():
        return []
    rows = []
    with open(path, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                rows.append({
                    "t": float(r["t"]), "x": float(r["x"]), "v": float(r["speed"]),
                    "gap": float(r["gap"]), "standby": int(r["standby"]),
                    "link": int(r.get("link", 1)), "frame": int(r["frame"]),
                })
            except (KeyError, ValueError):
                continue
    return rows


def analyse(seg: dict) -> dict:
    """把一段的时序压成几个读数。

    阶段判定用 link 位而不是 standby 位：车距有余量时断链也不制动，
    拿"是否在刹车"当链路判据会把"离线但没刹"读成"还连着"。
    """
    series = seg["series"]
    joined = next((r["t"] for r in series if r["link"] == 1), None)
    offline = None
    if joined is not None:
        offline = next((r["t"] for r in series if r["t"] > joined and r["link"] == 0), None)

    online = [r for r in series if r["link"] == 1]
    after = [r for r in series if offline is not None and r["t"] > offline]
    stopped = next((r["t"] for r in after if r["v"] <= 0.3), None)

    def min_gap(rows):
        vals = [r["gap"] for r in rows if r["gap"] >= 0.0]
        return min(vals) if vals else None

    avg_v = sum(r["v"] for r in online) / len(online) if online else None
    algo = seg.get("algo") or {}

    seg.update({
        "joined": joined,
        "offline": offline,
        "stopped": stopped,
        "min_gap_online": min_gap(online),
        "min_gap_after": min_gap(after),
        "avg_speed_online": avg_v,
        "end_speed": series[-1]["v"] if series else None,
        "duration": series[-1]["t"] if series else 0.0,
        "min_ttc": algo.get("最小TTC"),
        "max_accel": algo.get("最大加速度"),
        "reasons": algo.get("决策分布"),
        "state_frames": algo.get("状态帧"),
    })
    seg["kpis"] = build_kpis(seg)
    return seg


def build_kpis(seg: dict) -> list[dict]:
    if seg["offline"] is not None:
        stop_txt = (f"{seg['stopped']:.1f} s 停稳" if seg["stopped"] is not None
                    else f"末速 {fmt(seg['end_speed'], ' m/s')}")
        return [
            {"k": "断链时刻", "v": fmt(seg["offline"], " s"), "n": "算法进程退出、底座接管"},
            {"k": "断链前最小车距", "v": fmt(seg["min_gap_online"], " m"),
             "n": "算法进程控制时的最近距离"},
            {"k": "断链后最小车距", "v": fmt(seg["min_gap_after"], " m"),
             "n": "接管后车距不再收缩"},
            {"k": "接管结果", "v": stop_txt, "n": "兜底律的收敛结果"},
        ]
    return [
        {"k": "在线最小车距", "v": fmt(seg["min_gap_online"], " m"),
         "n": "算法进程控制下的最近距离"},
        {"k": "平均车速", "v": fmt(seg["avg_speed_online"], " m/s"),
         "n": f"期望 {seg['cruise']:.0f} m/s"},
        {"k": "最小 TTC", "v": fmt(seg["min_ttc"], " s"), "n": "算法侧估计的碰撞时间"},
        {"k": "状态帧", "v": fmt(seg["state_frames"], ""), "n": str(seg["reasons"] or "—")},
    ]


def fmt(v, unit="", nd=2):
    if v is None:
        return "—"
    if isinstance(v, int):
        return f"{v}{unit}"
    return f"{v:.{nd}f}{unit}"


def print_summary(segs: list[dict], out: Path) -> None:
    print("\n" + "=" * 68)
    print(f"  三段连播    共 {sum(len(s['frames']) for s in segs)} 帧    "
          f"{out / 'index.html'}")
    print("=" * 68)
    for s in segs:
        head = f"  {s['label']}  {s['title']}  ({s['duration']:.1f} s)"
        print(head)
        print(f"      在线最小车距 {fmt(s['min_gap_online'], ' m'):>8}"
              f"   平均车速 {fmt(s['avg_speed_online'], ' m/s'):>7}"
              f"   最小TTC {fmt(s['min_ttc'], ' s'):>6}")
        if s["offline"] is not None:
            print(f"      断链 {fmt(s['offline'], ' s'):>6}"
                  f"   断链后最小车距 {fmt(s['min_gap_after'], ' m'):>6}"
                  f"   末速 {fmt(s['end_speed'], ' m/s')}")
        print(f"      决策 {s['reasons']}")
    print("=" * 68)
    print()


# ---------------------------------------------------------------------------
# 页面
# ---------------------------------------------------------------------------

HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>ADASim Demo · 跟驰 → 切入 → 断链</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin: 0; background: #0d1117; color: #e6edf3;
         font: 14px/1.6 "Segoe UI", "Microsoft YaHei", system-ui, sans-serif; }
  .wrap { max-width: 1180px; margin: 0 auto; padding: 26px 24px 44px; }
  h1 { font-size: 20px; margin: 0 0 4px; font-weight: 600; }
  .sub { color: #8b949e; font-size: 13px; margin-bottom: 18px; }
  .chips { display: flex; gap: 8px; margin-bottom: 14px; flex-wrap: wrap; }
  .chip { background: #161b22; border: 1px solid #21262d; color: #8b949e;
          border-radius: 999px; padding: 5px 14px; cursor: pointer; font-size: 13px; }
  .chip:hover { border-color: #30363d; color: #e6edf3; }
  .chip.on { background: #12233d; border-color: #1f6feb; color: #79c0ff; }
  .grid { display: grid; grid-template-columns: 1.45fr 1fr; gap: 18px; }
  @media (max-width: 900px) { .grid { grid-template-columns: 1fr; } }
  .panel { background: #161b22; border: 1px solid #21262d; border-radius: 10px;
           padding: 14px; }
  .panel h2 { font-size: 12px; font-weight: 600; color: #8b949e; margin: 0 0 10px;
              letter-spacing: .05em; text-transform: uppercase; }
  img#screen { width: 100%; border-radius: 6px; display: block; background: #000;
               aspect-ratio: 1080 / 685; object-fit: contain; }
  .screen { position: relative; }
  /* 兜底接管要在画面上直接标出来：不标的话，观众会把底座的刹车
     当成算法的功劳——这正是第三段要对比的东西。 */
  .badge { position: absolute; left: 10px; top: 10px; padding: 4px 10px;
           border-radius: 6px; font-size: 12px; font-weight: 600;
           background: rgba(13,17,23,.86); border: 1px solid #30363d;
           color: #8b949e; display: none; }
  .badge.warn { color: #f0883e; border-color: #9e6a2f; }
  .ctrl { display: flex; align-items: center; gap: 10px; margin-top: 10px; }
  button { background: #21262d; color: #e6edf3; border: 1px solid #30363d;
           border-radius: 6px; padding: 5px 14px; cursor: pointer; font-size: 13px; }
  button:hover { background: #30363d; }
  input[type=range] { flex: 1; accent-color: #2f81f7; }
  .clock { font-variant-numeric: tabular-nums; color: #8b949e; font-size: 12px;
           min-width: 118px; text-align: right; }
  .kpis { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
  .kpi { background: #0d1117; border: 1px solid #21262d; border-radius: 8px;
         padding: 10px 12px; }
  .kpi .k { font-size: 12px; color: #8b949e; }
  .kpi .v { font-size: 18px; font-weight: 600; margin: 2px 0; color: #58a6ff;
            font-variant-numeric: tabular-nums; }
  .kpi .n { font-size: 11px; color: #6e7681; line-height: 1.45; }
  .seg-note { margin-top: 12px; font-size: 12.5px; color: #8b949e; line-height: 1.75;
              border-left: 2px solid #30363d; padding-left: 10px; }
  .charts { margin-top: 18px; display: grid; gap: 12px; }
  .chartbox { background: #161b22; border: 1px solid #21262d; border-radius: 10px;
              padding: 12px 14px; }
  .chartbox.on { border-color: #1f6feb; }
  .chartbox h3 { margin: 0 0 6px; font-size: 12.5px; color: #8b949e; font-weight: 500; }
  .chartbox.on h3 { color: #79c0ff; }
  canvas { width: 100%; height: 118px; display: block; }
  .legend { display: flex; gap: 18px; font-size: 12px; color: #8b949e; margin-top: 4px; }
  .dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%;
         margin-right: 5px; vertical-align: 1px; }
  .note { margin-top: 18px; color: #8b949e; font-size: 12.5px; line-height: 1.8; }
  code { background: #161b22; border: 1px solid #21262d; border-radius: 4px;
         padding: 1px 5px; font-size: 12px; }
</style>
</head>
<body>
<div class="wrap">
  <h1>ADASim · 跟驰 → 切入 → 断链</h1>
  <div class="sub">
    三段各录一次真实仿真，按顺序连播。前两段算法进程在线，第三段在同一个切入场景里
    让算法进程掉线——差别只来自"谁在控制"。
  </div>

  <div class="chips" id="chips"></div>

  <div class="grid">
    <div class="panel">
      <h2>仿真画面</h2>
      <div class="screen">
        <img id="screen" src="" alt="frame">
        <div class="badge" id="badge"></div>
      </div>
      <div class="ctrl">
        <button id="play">暂停</button>
        <input id="scrub" type="range" min="0" max="0" value="0" step="1">
        <span class="clock" id="clock">0.0 s</span>
      </div>
    </div>
    <div class="panel">
      <h2>本段读数</h2>
      <div class="kpis" id="kpis"></div>
      <div class="seg-note" id="segNote"></div>
    </div>
  </div>

  <div class="charts" id="charts"></div>

  <div class="legend" style="margin-top:10px">
    <span><i class="dot" style="background:#58a6ff"></i>车速 m/s</span>
    <span><i class="dot" style="background:#f0883e"></i>前车净距 m</span>
    <span><i class="dot" style="background:#8b949e"></i>灰底 = 算法进程离线</span>
    <span style="color:#6e7681">两条线量纲不同，各自按本段峰值归一</span>
  </div>

  <div class="note">
    <b>怎么看这页</b>　点上面的三段标签跳转，或直接让它自己连播。每段一张曲线，
    竖线是当前播放位置。第三段的灰底就是断链之后的区间：车速单调下降、车距不再收缩，
    说明兜底律接管生效。三段都是真跑出来的，不是拼接素材。
    复现：<code>__REPRO__</code>
  </div>
</div>
<script>
const D = __DATA__;
const SEG = D.segments;

// 全局帧号 → (段号, 段内帧号)，滑块才能跨段连续拖
const INDEX = [];
SEG.forEach((g, si) => g.frames.forEach((_, i) => INDEX.push([si, i])));

const img = document.getElementById('screen');
const scrub = document.getElementById('scrub');
const clock = document.getElementById('clock');
const btn = document.getElementById('play');
const kpis = document.getElementById('kpis');
const segNote = document.getElementById('segNote');
const chips = document.getElementById('chips');
const charts = document.getElementById('charts');
const badge = document.getElementById('badge');

let gi = 0, playing = true, curSeg = -1;
scrub.max = Math.max(0, INDEX.length - 1);

// 段标签
SEG.forEach((g, si) => {
  const b = document.createElement('button');
  b.className = 'chip';
  b.textContent = g.label + ' · ' + g.title;
  b.onclick = () => { jumpToSegment(si); };
  chips.appendChild(b);
});

// 每段一张画布
const canvases = SEG.map((g, si) => {
  const box = document.createElement('div');
  box.className = 'chartbox';
  const h = document.createElement('h3');
  h.textContent = g.label + '  ' + g.title + '　（' + g.duration.toFixed(1) + ' s）';
  const cv = document.createElement('canvas');
  cv.width = 1100; cv.height = 118;
  box.appendChild(h); box.appendChild(cv);
  charts.appendChild(box);
  return { box, cv, h };
});

function segStart(si) {
  let n = 0;
  for (let k = 0; k < si; k++) n += SEG[k].frames.length;
  return n;
}

function jumpToSegment(si) {
  playing = false; btn.textContent = '播放';
  show(segStart(si));
}

function nearestSeries(seg, t) {
  let best = 0, bestd = 1e9;
  for (let j = 0; j < seg.series.length; j++) {
    const d = Math.abs(seg.series[j].t - t);
    if (d < bestd) { bestd = d; best = j; }
  }
  return seg.series[best];
}

function show(g) {
  gi = Math.max(0, Math.min(INDEX.length - 1, g));
  const [si, fi] = INDEX[gi];
  const seg = SEG[si];
  img.src = seg.frames[fi];
  scrub.value = gi;

  // 帧与 100 ms 时序用挂钟时刻最近邻匹配；两种采样率不同，按下标硬对会错位
  const t = fi * seg.interval / 1000.0;
  const row = nearestSeries(seg, t);
  const tt = row ? row.t : t;
  clock.textContent = seg.label + ' · ' + tt.toFixed(1) + ' s';

  // 三态：在线（不打扰）/ 未接入 / 离线兜底。离线且没在刹也要说清楚，
  // 否则观众看到"车没减速"会以为兜底失效，其实只是车距还有余量。
  if (row && row.link === 1) {
    badge.style.display = 'none';
  } else if (row) {
    const pre = seg.joined !== null && row.t < seg.joined;
    badge.style.display = 'block';
    badge.className = row.standby === 1 ? 'badge warn' : 'badge';
    if (pre) {
      badge.textContent = '算法未接入 · 底座兜底接管';
    } else {
      badge.textContent = row.standby === 1
        ? '算法进程离线 · 兜底制动中'
        : '算法进程离线 · 车距有余量';
    }
  }

  if (si !== curSeg) {
    curSeg = si;
    [...chips.children].forEach((c, k) => c.classList.toggle('on', k === si));
    canvases.forEach((c, k) => c.box.classList.toggle('on', k === si));
    kpis.innerHTML = seg.kpis.map(o =>
      '<div class="kpi"><div class="k">' + o.k + '</div><div class="v">' + o.v +
      '</div><div class="n">' + o.n + '</div></div>').join('');
    segNote.textContent = seg.note;
  }
  canvases.forEach((c, k) => drawChart(c.cv, SEG[k], k === si, k === si ? tt : null));
}

btn.onclick = () => { playing = !playing; btn.textContent = playing ? '暂停' : '播放'; };
scrub.oninput = e => { playing = false; btn.textContent = '播放'; show(+e.target.value); };
setInterval(() => { if (playing) show((gi + 1) % INDEX.length); }, SEG[0].interval);

// 曲线：两条量纲不同的线共用一张图，各自按本段峰值归一。
// 目的不是精确读数，而是一眼看出"断链之后斜率变了"。
function drawChart(cv, seg, active, playT) {
  const ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height;
  const padL = 34, padR = 10, padT = 8, padB = 16;
  const S = seg.series;
  if (!S.length) return;

  const tMax = Math.max(1e-6, S[S.length - 1].t);
  const vMax = Math.max(1, ...S.map(r => r.v));
  const gMax = Math.max(1, ...S.map(r => Math.max(0, r.gap)));
  const X = t => padL + (W - padL - padR) * (t / tMax);
  const Yv = v => padT + (H - padT - padB) * (1 - v / vMax);
  const Yg = g => padT + (H - padT - padB) * (1 - g / gMax);

  ctx.clearRect(0, 0, W, H);

  // 离线区间打灰底：比一条竖线更能说明"这段时间没人管"
  if (seg.offline !== null) {
    const x = X(seg.offline);
    ctx.fillStyle = 'rgba(139,148,158,0.13)';
    ctx.fillRect(x, padT, W - padR - x, H - padT - padB);
    ctx.strokeStyle = '#8b949e'; ctx.setLineDash([4, 4]); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, H - padB); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = '#8b949e'; ctx.font = '10px sans-serif'; ctx.textAlign = 'left';
    ctx.fillText('断链', x + 4, padT + 9);
  }

  ctx.strokeStyle = active ? '#30363d' : '#21262d'; ctx.lineWidth = 1;
  ctx.beginPath();
  for (let k = 0; k <= 2; k++) {
    const y = padT + (H - padT - padB) * k / 2;
    ctx.moveTo(padL, y); ctx.lineTo(W - padR, y);
  }
  ctx.stroke();

  const line = (fn, color, width) => {
    ctx.strokeStyle = color; ctx.lineWidth = width;
    ctx.beginPath();
    S.forEach((r, k) => {
      const x = X(r.t), y = fn(r);
      k ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    });
    ctx.stroke();
  };
  line(r => Yv(r.v), active ? '#58a6ff' : '#3b6b9e', 1.7);
  line(r => Yg(Math.max(0, r.gap)), active ? '#f0883e' : '#8a5a2f', 1.7);

  ctx.fillStyle = '#6e7681'; ctx.font = '10px sans-serif';
  ctx.textAlign = 'right';
  ctx.fillText(vMax.toFixed(0), padL - 5, padT + 8);
  ctx.fillText('0', padL - 5, H - padB + 3);
  ctx.textAlign = 'center';
  for (let k = 0; k <= 4; k++) {
    const t = tMax * k / 4;
    ctx.fillText(t.toFixed(1), X(t), H - 4);
  }

  // 播放头
  if (playT !== null) {
    const x = X(playT);
    ctx.strokeStyle = '#f85149'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, H - padB); ctx.stroke();
  }
}

show(0);
</script>
</body>
</html>
"""


def render_html(segs: list[dict], out: Path, repro: str) -> str:
    payload_segments = []
    for seg in segs:
        # 帧内联成 data URI。这份页面是要发给别人看的（导师、答辩评委），
        # 单个 html 文件不会因为漏了 frames 目录就变成一排裂图标。
        frames_uri = []
        for p in seg["frames"]:
            path = Path(p)
            if path.exists():
                frames_uri.append("data:image/png;base64,"
                                  + base64.b64encode(path.read_bytes()).decode("ascii"))
            else:
                frames_uri.append("")
        payload_segments.append({
            "id": seg["id"],
            "label": seg["label"],
            "title": seg["title"],
            "note": seg["note"],
            "interval": seg["interval"],
            "duration": round(seg["duration"], 2),
            "frames": frames_uri,
            "series": [{"t": round(r["t"], 2), "v": round(r["v"], 2),
                        "gap": round(r["gap"], 2), "link": r["link"],
                        "standby": r["standby"]}
                       for r in seg["series"]],
            "offline": seg["offline"],
            "joined": seg["joined"],
            "kpis": seg["kpis"],
        })

    data = json.dumps({"segments": payload_segments}, ensure_ascii=False)
    return HTML.replace("__DATA__", data).replace("__REPRO__", repro)


def strip_frames(segs: list[dict]) -> list[dict]:
    """report.json 里不留帧路径以外的重字段，也不留 base64。"""
    out = []
    for s in segs:
        d = {k: v for k, v in s.items() if k != "frames"}
        d["帧数"] = len(s["frames"])
        out.append(d)
    return out


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="ADASim 三段连播 Demo")
    p.add_argument("--out", default=str(OUT_DEFAULT))
    p.add_argument("--scenario", default=None,
                   help="只跑单个场景（退回单段模式），默认跑内置三段")
    p.add_argument("--only", default=None,
                   help="只跑某一段：follow / cutin / drop")
    p.add_argument("--frames", type=int, default=16, help="每段录制帧数")
    p.add_argument("--interval", type=int, default=700, help="帧间隔，毫秒")
    p.add_argument("--drop-at", type=float, default=None,
                   help="第三段算法进程在第几秒退出，缺省 5.0")
    p.add_argument("--startup", type=float, default=2.0, help="等底座监听的秒数")
    p.add_argument("--rebuild", action="store_true",
                   help="复用上一轮的帧与时序，只重排页面")
    args = p.parse_args(argv)

    # 剧本：默认三段；--scenario 退回单段；--only 只跑指定段
    if args.scenario:
        segs_spec = [{
            "id": "single", "label": "单场景",
            "scenario": args.scenario, "online": args.drop_at,
            "note": "单场景运行。",
        }]
    else:
        segs_spec = [dict(s) for s in SEGMENTS]
        if args.drop_at is not None:
            for s in segs_spec:
                if s["id"] == "drop":
                    s["online"] = args.drop_at
        if args.only:
            segs_spec = [s for s in segs_spec if s["id"] == args.only]
            if not segs_spec:
                print(f"没有这一段：{args.only}（可选 "
                      f"{'/'.join(s['id'] for s in SEGMENTS)}）", file=sys.stderr)
                return 2

    for s in segs_spec:
        path = ROOT / s["scenario"]
        if not path.exists():
            print(f"场景不存在: {path}", file=sys.stderr)
            return 2

    out = Path(args.out).resolve()

    if args.rebuild:
        # 只重排页面，不重跑仿真：数据在上一轮的 segments/ 里
        if not out.exists():
            print(f"没有可复用的产物：{out}\n先不带 --rebuild 跑一次", file=sys.stderr)
            return 2
        run_meta = {}
        if (out / "run.json").exists():
            run_meta = json.loads((out / "run.json").read_text(encoding="utf-8"))
        segs = []
        for spec in segs_spec:
            spec.update(run_meta.get(spec["id"], {}))
            seg = load_segment(spec, out)
            if seg is None:
                print(f"段 {spec['id']} 没有产物，先完整跑一次", file=sys.stderr)
                return 2
            segs.append(analyse(seg))
        return finish(segs, out, args)

    base = find_base()
    if base is None:
        print("找不到 adasim 可执行文件，请先构建：\n"
              "    cmake --build build/qt --target adasim", file=sys.stderr)
        return 2
    warn = check_qt_runtime(base)
    if warn:
        print(warn, file=sys.stderr)
        return 2

    if out.exists():
        shutil.rmtree(out)          # 上一轮的段目录会混进来
    out.mkdir(parents=True)

    print(f"共 {len(segs_spec)} 段，每段 {args.frames} 帧 × {args.interval} ms\n")
    segs: list[dict] = []
    run_meta = {}
    for spec in segs_spec:
        seg = run_segment(spec, out, base, args)
        if seg is None:
            return 3
        segs.append(analyse(seg))
        # 记下本段实际用的节拍，--rebuild 时按这份记录还原，而不是按默认参数
        run_meta[spec["id"]] = {"interval": seg["interval"],
                                "frames": len(seg["frames"])}
    (out / "run.json").write_text(
        json.dumps(run_meta, ensure_ascii=False, indent=2), encoding="utf-8")

    return finish(segs, out, args)


def finish(segs: list[dict], out: Path, args) -> int:
    repro = "python tools/demo.py"
    if args.scenario:
        repro += f" --scenario {args.scenario}"
    if args.only:
        repro += f" --only {args.only}"

    (out / "index.html").write_text(render_html(segs, out, repro), encoding="utf-8")
    (out / "report.json").write_text(
        json.dumps(strip_frames(segs), ensure_ascii=False, indent=2), encoding="utf-8")

    print_summary(segs, out)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n已中断")
        sys.exit(130)
