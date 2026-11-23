"""一条命令跑通整个 Demo：底座 + 算法进程 + 断链兜底，产出可播放的页面。

为什么需要这个脚本
------------------
手工演示这条链路至少要敲四条命令、掐两次时序：先起底座，再起算法进程，
中途把算法进程停掉，最后收数据。掐错一次时序，看到的就是"算法没连上"
或者"断链在录完之后"，而这两种情况的画面看起来都很正常。脚本把时序固定下来，
演示就变成可重复的一件事，而不是"上次跑的时候没问题"。

用法
----
    python tools/demo.py                     # 默认切入场景，约 12 秒
    python tools/demo.py --scenario scenarios/following_lead.json
    python tools/demo.py --seconds 16 --interval 600
    python tools/demo.py --keep-online       # 不断链，只看闭环跟驰

产物（默认 output/demo/）
    frames/frame_XX.png   同一次仿真的连续截图
    timeseries.csv        100 ms 采样的仿真时间/车速/车距/兜底标志
    algo.json             算法进程汇总（在线阶段的指标）
    index.html            播放器 + 曲线 + 结论，双击即可看
"""

from __future__ import annotations

import argparse
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

    与其让人对着"闪一下就没了"猜半天，不如在这里提前说清楚：
    把 Qt 的 bin 目录加进 PATH 再跑。
    """
    if os.name != "nt":
        return None
    qt = os.environ.get("QTDIR") or os.environ.get("QT_ROOT")
    bin_dir = Path(qt) / "bin" if qt else None
    if bin_dir and (bin_dir / "Qt6Core.dll").exists():
        return None
    # PATH 里已经有 Qt 也算就绪（最常见的情形）
    for p in os.environ.get("PATH", "").split(os.pathsep):
        try:
            if (Path(p) / "Qt6Core.dll").exists():
                return None
        except OSError:
            continue
    return (f"没在 PATH 里找到 Qt6Core.dll。请先把 Qt 的 bin 目录加入 PATH，例如：\n"
            f"    export PATH=\"/path/to/Qt/6.x/mingw_64/bin:/path/to/Qt/Tools/mingw1310_64/bin:$PATH\"\n"
            f"底座路径: {base}")


# ---------------------------------------------------------------------------
# 跑一次演示
# ---------------------------------------------------------------------------

def run_demo(args) -> int:
    base = find_base()
    if base is None:
        print("找不到 adasim 可执行文件，请先构建：\n"
              "    cmake --build build/qt --target adasim", file=sys.stderr)
        return 2
    warn = check_qt_runtime(base)
    if warn:
        print(warn, file=sys.stderr)
        return 2

    out = Path(args.out).resolve()
    if out.exists():
        shutil.rmtree(out)          # 上一轮的帧会混进来，看起来像"多录了两帧"
    frames_dir = out / "frames"
    frames_dir.mkdir(parents=True)

    scenario = Path(args.scenario)
    if not scenario.is_absolute():
        scenario = ROOT / scenario
    if not scenario.exists():
        print(f"场景不存在: {scenario}", file=sys.stderr)
        return 2
    meta = json.loads(scenario.read_text(encoding="utf-8"))
    cruise = args.cruise if args.cruise is not None else meta.get("egoSpeed", 10.0)

    total = args.frames * args.interval / 1000.0
    online = total if args.keep_online else args.online_seconds
    online = min(online, total - 2.0)   # 至少留 2 s 给"断链之后"的镜头

    ts_path = out / "timeseries.csv"
    algo_path = out / "algo.json"
    log_base = out / "base.log"

    print(f"场景      : {meta.get('name', scenario.stem)}  ({scenario.name})")
    print(f"录制      : {args.frames} 帧 × {args.interval} ms ≈ {total:.1f} s")
    print(f"算法在线  : {online:.1f} s" +
          ("（全程在线）" if args.keep_online else "，之后断链，观察兜底制动"))
    print()

    base_cmd = [
        str(base), str(scenario),
        "--record", str(frames_dir),
        "--frames", str(args.frames),
        "--interval", str(args.interval),
        "--metrics", str(ts_path),
    ]
    with open(log_base, "w", encoding="utf-8") as log:
        proc = subprocess.Popen(base_cmd, stdout=log, stderr=subprocess.STDOUT)
        try:
            # 底座要先把网关监听起来，算法进程才有地方连。
            time.sleep(args.startup)
            if proc.poll() is not None:
                print("底座启动即退出，看 base.log：\n" + log_base.read_text(encoding="utf-8"),
                      file=sys.stderr)
                return 3

            node_cmd = [
                sys.executable, str(ROOT / "python" / "run.py"),
                "--duration", f"{online:.1f}",
                "--cruise", f"{cruise:.1f}",
                "--quiet",
                "--summary-json", str(algo_path),
            ]
            node = subprocess.run(node_cmd, cwd=ROOT, capture_output=True, text=True,
                                  encoding="utf-8", errors="replace")
            if node.stdout.strip():
                print(node.stdout.strip().splitlines()[-1])

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

    frames = sorted(frames_dir.glob("frame_*.png"))
    if not frames:
        print("没有拿到任何帧，检查 base.log：\n" + log_base.read_text(encoding="utf-8"),
              file=sys.stderr)
        return 3

    series = read_series(ts_path)
    algo = json.loads(algo_path.read_text(encoding="utf-8")) if algo_path.exists() else {}

    report = build_report(scenario, meta, series, algo, args, frames, cruise)
    (out / "index.html").write_text(render_html(report), encoding="utf-8")
    (out / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    print_summary(report, out)
    return 0


# ---------------------------------------------------------------------------
# 取数与结论
# ---------------------------------------------------------------------------

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


def build_report(scenario, meta, series, algo, args, frames, cruise) -> dict:
    # 三个阶段：底座先起（link=0，算法未接入）→ 算法接入（link=1）→ 断链（link=0）。
    # 只用 standby 位判"断链"是错的：车距有余量时断链也不制动，会被读成"还连着"。
    joined = next((r["t"] for r in series if r["link"] == 1), None)
    off = None
    if joined is not None:
        off = next((r["t"] for r in series if r["t"] > joined and r["link"] == 0), None)

    online = [r for r in series if r["link"] == 1]
    after = [r for r in series if off is not None and r["t"] > off]

    stopped = next((r["t"] for r in after if r["v"] <= 0.3), None)

    def min_gap(rows):
        vals = [r["gap"] for r in rows if r["gap"] >= 0.0]
        return min(vals) if vals else None

    return {
        "场景": meta.get("name", scenario.stem),
        "场景文件": str(scenario.relative_to(ROOT)) if scenario.is_relative_to(ROOT) else str(scenario),
        "期望车速": cruise,
        "仿真时长": series[-1]["t"] if series else 0.0,
        "帧数": len(frames),
        "帧间隔": args.interval,
        "接入时刻": joined,
        "断链时刻": off,
        "停稳时刻": stopped,
        "在线最小车距": min_gap(online),
        "断链后最小车距": min_gap(after),
        "断链时车速": next((r["v"] for r in after), None),
        "末速": series[-1]["v"] if series else None,
        "算法汇总": algo,
        "采样": series,
    }


def fmt(v, unit="", nd=2):
    if v is None:
        return "—"
    if isinstance(v, int):
        return f"{v}{unit}"
    return f"{v:.{nd}f}{unit}"


def print_summary(rep: dict, out: Path) -> None:
    algo = rep["算法汇总"]
    print("\n" + "=" * 62)
    print(f"  场景 {rep['场景']}    仿真 {rep['仿真时长']:.1f} s    "
          f"{rep['帧数']} 帧")
    print("=" * 62)
    if rep["接入时刻"] is not None:
        print(f"  算法接入          {rep['接入时刻']:.1f} s")
    print(f"  在线最小车距      {fmt(rep['在线最小车距'], ' m')}")
    if rep["断链时刻"] is not None:
        print(f"  断链时刻          {rep['断链时刻']:.1f} s（算法进程退出）")
        print(f"  兜底后最小车距    {fmt(rep['断链后最小车距'], ' m')}")
        if rep["停稳时刻"] is not None:
            print(f"  停稳时刻          {rep['停稳时刻']:.1f} s")
        print(f"  断链时 / 末车速   {fmt(rep['断链时车速'], ' m/s')} → "
              f"{fmt(rep['末速'], ' m/s')}")
    if algo:
        print(f"  算法侧最小 TTC    {fmt(algo.get('最小TTC'), ' s')}")
        print(f"  决策分布          {algo.get('决策分布')}")
    print("=" * 62)
    print(f"  打开看效果：{out / 'index.html'}")
    print()


# ---------------------------------------------------------------------------
# 页面
# ---------------------------------------------------------------------------

def render_html(rep: dict) -> str:
    payload = {
        "frames": [f"frames/frame_{i:02d}.png" for i in range(rep["帧数"])],
        "interval": rep["帧间隔"],
        "series": [{"t": round(r["t"], 2), "v": round(r["v"], 2),
                    "gap": round(r["gap"], 2), "link": r.get("link", 1)}
                   for r in rep["采样"]],
        "joined": rep["接入时刻"],
        "offline": rep["断链时刻"],
        "stopped": rep["停稳时刻"],
        # 每帧对应的挂钟时刻（帧 i 在第 i×interval 毫秒抓取）。
        # 用它去最近邻匹配 100 ms 采样的时序，而不是用下标硬对——
        # 两种采样率不同，按下标配对会把第 10 帧的读数安到第 3 秒上。
        "frameTimes": [round(i * rep["帧间隔"] / 1000.0, 2) for i in range(rep["帧数"])],
    }
    data = json.dumps(payload, ensure_ascii=False)

    if rep["断链时刻"] is not None:
        stop_txt = (f"{rep['停稳时刻']:.1f} s 停稳" if rep["停稳时刻"] is not None
                    else f"末速 {fmt(rep['末速'], ' m/s')}")
        kpi = [
            ("在线最小车距", fmt(rep["在线最小车距"], " m"), "算法进程控制时的最近距离"),
            ("断链时刻", fmt(rep["断链时刻"], " s"), "算法进程退出、底座接管"),
            ("兜底后最小车距", fmt(rep["断链后最小车距"], " m"), "接管后车距不再收缩到危险区"),
            ("接管结果", stop_txt, "兜底律的收敛结果"),
        ]
    else:
        kpi = [
            ("在线最小车距", fmt(rep["在线最小车距"], " m"), "算法进程控制下的最近距离"),
            ("算法接入", fmt(rep["接入时刻"], " s"), "算法进程连上底座"),
            ("末速", fmt(rep["末速"], " m/s"), "录制结束时的车速"),
            ("仿真时长", fmt(rep["仿真时长"], " s"), "本次录制的总时长"),
        ]
    cards = "\n".join(
        f'<div class="kpi"><div class="k">{k}</div><div class="v">{v}</div>'
        f'<div class="n">{n}</div></div>' for k, v, n in kpi)

    if rep["断链时刻"] is not None:
        sub = ("同一次仿真的连续截图（每 %d ms 一张）。前半段由算法进程闭环控制，"
               "算法进程退出后由底座兜底接管——画面里能看到车自己减速。" % rep["帧间隔"])
        foot = ("拖动滑块看任意时刻的画面；曲线上的灰色竖线是算法进程退出的时刻，"
                "之后车速单调下降、车距不再收缩，说明兜底律接管生效。")
    else:
        sub = ("同一次仿真的连续截图（每 %d ms 一张），算法进程全程在线闭环控制。"
               % rep["帧间隔"])
        foot = "拖动滑块看任意时刻的画面；曲线是本次仿真的车速与车距时序。"

    return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>ADASim Demo · {rep['场景']}</title>
<style>
  :root {{ color-scheme: dark; }}
  * {{ box-sizing: border-box; }}
  body {{ margin: 0; background: #0d1117; color: #e6edf3;
         font: 14px/1.6 "Segoe UI", "Microsoft YaHei", system-ui, sans-serif; }}
  .wrap {{ max-width: 1180px; margin: 0 auto; padding: 28px 24px 48px; }}
  h1 {{ font-size: 20px; margin: 0 0 4px; font-weight: 600; }}
  .sub {{ color: #8b949e; font-size: 13px; margin-bottom: 22px; }}
  .grid {{ display: grid; grid-template-columns: 1.45fr 1fr; gap: 20px; }}
  @media (max-width: 900px) {{ .grid {{ grid-template-columns: 1fr; }} }}
  .panel {{ background: #161b22; border: 1px solid #21262d; border-radius: 10px;
            padding: 14px; }}
  .panel h2 {{ font-size: 13px; font-weight: 600; color: #8b949e; margin: 0 0 10px;
               letter-spacing: .04em; text-transform: uppercase; }}
  img#screen {{ width: 100%; border-radius: 6px; display: block; background: #000;
                aspect-ratio: 1080 / 685; object-fit: contain; }}
  .ctrl {{ display: flex; align-items: center; gap: 10px; margin-top: 10px; }}
  button {{ background: #21262d; color: #e6edf3; border: 1px solid #30363d;
            border-radius: 6px; padding: 5px 14px; cursor: pointer; font-size: 13px; }}
  button:hover {{ background: #30363d; }}
  input[type=range] {{ flex: 1; accent-color: #2f81f7; }}
  .clock {{ font-variant-numeric: tabular-nums; color: #8b949e; font-size: 12px;
            min-width: 96px; text-align: right; }}
  .kpis {{ display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }}
  .kpi {{ background: #0d1117; border: 1px solid #21262d; border-radius: 8px;
          padding: 10px 12px; }}
  .kpi .k {{ font-size: 12px; color: #8b949e; }}
  .kpi .v {{ font-size: 19px; font-weight: 600; margin: 2px 0; color: #58a6ff;
             font-variant-numeric: tabular-nums; }}
  .kpi .n {{ font-size: 11px; color: #6e7681; line-height: 1.45; }}
  .chart {{ margin-top: 20px; }}
  canvas {{ width: 100%; height: 200px; display: block; }}
  .legend {{ display: flex; gap: 18px; font-size: 12px; color: #8b949e; margin-top: 6px; }}
  .dot {{ display: inline-block; width: 8px; height: 8px; border-radius: 50%;
          margin-right: 5px; vertical-align: 1px; }}
  .note {{ margin-top: 20px; color: #8b949e; font-size: 12.5px; line-height: 1.8; }}
  code {{ background: #161b22; border: 1px solid #21262d; border-radius: 4px;
          padding: 1px 5px; font-size: 12px; }}
</style>
</head>
<body>
<div class="wrap">
  <h1>ADASim · {rep['场景']}</h1>
  <div class="sub">
    {sub}
    场景 <b>{rep['场景']}</b>，期望车速 {rep['期望车速']:.0f} m/s。
  </div>

  <div class="grid">
    <div class="panel">
      <h2>仿真画面</h2>
      <img id="screen" src="" alt="frame">
      <div class="ctrl">
        <button id="play">暂停</button>
        <input id="scrub" type="range" min="0" max="0" value="0" step="1">
        <span class="clock" id="clock">0.0 s</span>
      </div>
    </div>
    <div class="panel">
      <h2>关键读数</h2>
      <div class="kpis">{cards}</div>
      <div class="note" style="margin-top:12px">
        数据来源：底座 100 ms 采样的 <code>timeseries.csv</code>
        与算法进程汇总 <code>algo.json</code>。
      </div>
    </div>
  </div>

  <div class="panel chart">
    <h2>车速与车距</h2>
    <canvas id="cv" width="1120" height="200"></canvas>
    <div class="legend">
      <span><i class="dot" style="background:#58a6ff"></i>车速 m/s</span>
      <span><i class="dot" style="background:#f0883e"></i>前车净距 m</span>
      <span><i class="dot" style="background:#8b949e"></i>算法进程在线 / 断链</span>
    </div>
  </div>

  <div class="note">
    <b>怎么看这页</b>　{foot}
    复现：<code>python tools/demo.py --scenario {rep['场景文件']}</code>
  </div>
</div>
<script>
const D = {data};
let i = 0, playing = true;
const img = document.getElementById('screen');
const scrub = document.getElementById('scrub');
const clock = document.getElementById('clock');
const btn = document.getElementById('play');
scrub.max = D.frames.length - 1;

function show(k) {{
  i = Math.max(0, Math.min(D.frames.length - 1, k));
  img.src = D.frames[i];
  scrub.value = i;
  const ft = D.frameTimes ? D.frameTimes[i] : 0;
  let best = 0, bestd = 1e9;
  for (let j = 0; j < D.series.length; j++) {{
    const d = Math.abs(D.series[j].t - ft);
    if (d < bestd) {{ bestd = d; best = j; }}
  }}
  const s = D.series[best];
  clock.textContent = (s ? s.t.toFixed(1) : ft.toFixed(1)) + ' s';
}}
btn.onclick = () => {{ playing = !playing; btn.textContent = playing ? '暂停' : '播放'; }};
scrub.oninput = e => {{ playing = false; btn.textContent = '播放'; show(+e.target.value); }};
setInterval(() => {{ if (playing) show((i + 1) % D.frames.length); }}, D.interval);
show(0);

// 曲线：两条量纲不同的线共用一张图，各自按自身峰值归一，
// 目的不是精确读数，而是一眼看出"断链前后斜率变了"。
const cv = document.getElementById('cv'), ctx = cv.getContext('2d');
const S = D.series;
const padL = 38, padR = 12, padT = 12, padB = 22;
const W = cv.width, H = cv.height;
const tMax = Math.max(1e-6, S.length ? S[S.length - 1].t : 1);
const vMax = Math.max(1, ...S.map(r => r.v));
const gMax = Math.max(1, ...S.map(r => Math.max(0, r.gap)));
const X = t => padL + (W - padL - padR) * (t / tMax);
const Yv = v => padT + (H - padT - padB) * (1 - v / vMax);
const Yg = g => padT + (H - padT - padB) * (1 - g / gMax);

ctx.strokeStyle = '#21262d'; ctx.lineWidth = 1;
ctx.beginPath();
for (let k = 0; k <= 4; k++) {{
  const y = padT + (H - padT - padB) * k / 4;
  ctx.moveTo(padL, y); ctx.lineTo(W - padR, y);
}}
ctx.stroke();
ctx.fillStyle = '#6e7681'; ctx.font = '11px sans-serif'; ctx.textAlign = 'right';
ctx.fillText(vMax.toFixed(1), padL - 6, padT + 8);
ctx.fillText('0', padL - 6, H - padB + 4);

if (D.offline !== null) {{
  const x = X(D.offline);
  ctx.fillStyle = 'rgba(139,148,158,0.10)';
  ctx.fillRect(x, padT, W - padR - x, H - padT - padB);
  ctx.strokeStyle = '#8b949e'; ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, H - padB); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = '#8b949e'; ctx.textAlign = 'left';
  ctx.fillText('断链 · 底座接管', x + 5, padT + 10);
}}

function line(fn, color) {{
  ctx.strokeStyle = color; ctx.lineWidth = 1.8; ctx.beginPath();
  S.forEach((r, k) => {{ const x = X(r.t), y = fn(r); k ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }});
  ctx.stroke();
}}
line(r => Yv(r.v), '#58a6ff');
line(r => Yg(Math.max(0, r.gap)), '#f0883e');
ctx.fillStyle = '#6e7681'; ctx.textAlign = 'center';
for (let k = 0; k <= 5; k++) {{
  const t = tMax * k / 5;
  ctx.fillText(t.toFixed(1), X(t), H - 6);
}}
</script>
</body>
</html>
"""


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="一条命令跑通 ADASim 端到端 Demo")
    p.add_argument("--scenario", default="scenarios/cut_in_brake.json")
    p.add_argument("--out", default=str(OUT_DEFAULT))
    p.add_argument("--frames", type=int, default=16, help="录制帧数")
    p.add_argument("--interval", type=int, default=700, help="帧间隔，毫秒")
    p.add_argument("--online-seconds", type=float, default=6.0,
                   help="算法进程在线时长，之后断链")
    p.add_argument("--keep-online", action="store_true", help="算法进程全程在线")
    p.add_argument("--cruise", type=float, default=None, help="期望车速，缺省取场景 egoSpeed")
    p.add_argument("--startup", type=float, default=2.0, help="等底座监听的秒数")
    args = p.parse_args(argv)
    try:
        return run_demo(args)
    except KeyboardInterrupt:
        print("\n已中断")
        return 130


if __name__ == "__main__":
    sys.exit(main())
