"""贝叶斯失效搜索：在场景参数空间里找"最容易出事"的那一组。

**问题形式**

    输入  参数向量 x（自车初速、初始间距、切入时刻、附着系数、感知噪声）
    评估  跑一遍闭环仿真，得到 (是否碰撞, 最小 TTC, 最小净距, 峰值加速度)
    目标  最大化"失效严重度" severity(x)

**为什么不是网格搜索或随机搜索**

搜索空间是 5 维连续，网格要覆盖到有意义的分辨率需要上万次评估；
随机搜索在 100 次预算下对 5 维问题的覆盖极其稀疏。失效边界通常是
参数空间里的一张薄曲面——绝大多数采样点落在"安全"或"必然碰撞"的
两侧平原上，只有边界附近才有信息量。贝叶斯优化用已评估的点拟合
代理模型，把下一次评估主动放到"最可能改进"的位置，正是为这种
"评估贵、边界薄"的形态设计的。

**severity 的构造**

    collided  → 1.0 + 0.5 · clamp(impactSpeed / 10, 0, 1)   ∈ [1.0, 1.5]
    未碰撞    → 1 / (1 + max(minTtc, 0))                    ∈ (0, 1]

两段在 TTC→0 处都取值 1.0，因此是连续的——这一点很重要：不连续的目标
会让 GP 在跳变处拟合出一条陡坡，采集函数被它带偏，把预算全花在跳变点两侧。

碰撞深度为什么用"撞击时的接近速度"而不是"穿透深度"：碰撞后立即终止
仿真以节省预算，穿透深度永远只是刚接触那一帧的值，在碰撞区间里退化成
常数——搜索器在那里拿不到任何梯度，后半程等于随机游走。接近速度能区分
"轻轻蹭上"与"高速追尾"，物理上也更接近损伤严重度。

**加速度为什么不进目标函数**

最大纵向加速度与失效程度不同向：一次平稳的紧急制动可以有 6 m/s²，
而一次轻微剐蹭可能只有 2 m/s²。把它塞进同一个标量会引入矛盾信号。
这里照开题的约定把它作为**报告指标**随结果一并输出，不参与优化。
"""

from __future__ import annotations

import json
import math
import os
import random
import subprocess
from dataclasses import dataclass, field
from typing import Any

from .gp import GaussianProcess, expected_improvement, fit_best
from .template import Scenario

#: 评估进程的默认位置（相对项目根目录）。
DEFAULT_EXE = os.path.join("build", "qt", "bin", "adasim_batch.exe")


@dataclass
class Metrics:
    """一次仿真的原始指标。字段与 `adasim_batch` 的响应一一对应。"""

    collided: bool = False
    min_ttc: float = -1.0
    min_gap: float = -1.0
    max_abs_accel: float = 0.0
    impact_speed: float = 0.0
    travelled: float = 0.0
    steps: int = 0
    wall_ms: float = 0.0

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> "Metrics":
        return cls(
            collided=bool(data.get("collided", False)),
            min_ttc=float(data.get("minTtc", -1.0)),
            min_gap=float(data.get("minGap", -1.0)),
            max_abs_accel=float(data.get("maxAbsAccel", 0.0)),
            impact_speed=float(data.get("impactSpeed", 0.0)),
            travelled=float(data.get("travelled", 0.0)),
            steps=int(data.get("steps", 0)),
            wall_ms=float(data.get("wallMs", 0.0)),
        )


#: 撞击接近速度的参考尺度，m/s。取 10 m/s（36 km/h）为"满分"：
#: 这是城市道路追尾里足以造成人员伤害的量级，再往上区分意义不大。
_IMPACT_REFERENCE = 10.0


def severity(m: Metrics) -> float:
    """失效严重度，越大越危险。定义见模块文档。"""
    if m.collided:
        # 用撞击时的接近速度而不是穿透深度：碰撞后立即终止仿真，
        # 穿透深度永远只是"刚接触"那一帧的值，在碰撞区间里是个常数。
        return 1.0 + 0.5 * min(1.0, max(0.0, m.impact_speed) / _IMPACT_REFERENCE)
    if m.min_ttc < 0.0:
        return 0.0      # 全程没有接近过任何目标
    return 1.0 / (1.0 + max(0.0, m.min_ttc))


@dataclass
class Trial:
    """一次评估的记录。参数用有序 dict 保存，便于落盘成人读的表。"""

    params: dict[str, float]
    metrics: Metrics
    score: float
    phase: str          # baseline | init | bo

    def as_row(self) -> dict[str, Any]:
        row: dict[str, Any] = dict(self.params)
        row.update({
            "severity": round(self.score, 4),
            "collided": self.metrics.collided,
            "minTtc": round(self.metrics.min_ttc, 3),
            "minGap": round(self.metrics.min_gap, 3),
            "impactSpeed": round(self.metrics.impact_speed, 3),
            "maxAbsAccel": round(self.metrics.max_abs_accel, 3),
            "phase": self.phase,
        })
        return row


class BatchEvaluator:
    """常驻评估进程的客户端。

    每轮把参数写进同一份场景文件再交给进程评估，而不是把场景内联进请求：
    好处是**每一轮跑了什么都有据可查**。若只放在内存里，事后想复现
    "第 37 次评估为什么撞了"就只能重跑整个搜索。
    """

    def __init__(self, exe: str = DEFAULT_EXE, scenario: Scenario | None = None,
                 root: str = ".", work_path: str | None = None,
                 noise_seed: int = 1, timeout: float = 30.0) -> None:
        self.exe = exe
        self.root = root
        self.noise_seed = noise_seed
        self.timeout = timeout
        self.scenario = scenario
        self.work_path = work_path or os.path.join(
            os.getcwd(), "ai", "out", "_eval_tmp.json")
        self.work_path = os.path.abspath(self.work_path)

        self._proc: subprocess.Popen[str] | None = None
        self.evaluations = 0
        self.cache: dict[tuple[float, ...], Metrics] = {}

        if not os.path.isfile(exe):
            raise FileNotFoundError(
                f"找不到评估进程 {exe}。请先构建（cmake --build build/qt），"
                "或用 --exe 指定路径。")

    # ---- 进程生命周期 ----

    def _ensure_proc(self) -> subprocess.Popen[str]:
        if self._proc is not None and self._proc.poll() is None:
            return self._proc

        # 显式 utf-8：Windows 默认编码是 gbk，中文错误信息会在管道上炸掉。
        self._proc = subprocess.Popen(
            [self.exe, "--root", self.root],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            bufsize=1,          # 行缓冲：写完就能读到响应
        )
        return self._proc

    def close(self) -> None:
        if self._proc is None:
            return
        try:
            if self._proc.stdin:
                self._proc.stdin.close()
            self._proc.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            self._proc.kill()
        self._proc = None

    def __enter__(self) -> "BatchEvaluator":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # ---- 评估 ----

    def evaluate(self, params: dict[str, float], scenario: Scenario | None = None) -> Metrics:
        """给一组参数，返回指标。相同参数直接命中缓存。

        缓存不只是省时间：贝叶斯优化会在极优点附近反复试探，浮点相同的
        参数若重复评估，GP 会看到"同一输入对应不同输出"（因为进程状态
        并非完全无状态），进而把噪声方差估高、模型变钝。
        """
        key = tuple(round(params[k], 9) for k in sorted(params))
        if key in self.cache:
            return self.cache[key]

        base = scenario or self.scenario
        if base is None:
            raise ValueError("没有可用的场景模板")

        applied = base.apply_parameters(params)
        applied.save(self.work_path)

        request = {
            "path": self.work_path,
            "noise": float(params.get("perception_noise", 0.0)),
            "seed": self.noise_seed,
            "closedLoop": True,
        }

        proc = self._ensure_proc()
        assert proc.stdin is not None and proc.stdout is not None
        proc.stdin.write(json.dumps(request, ensure_ascii=False) + "\n")
        proc.stdin.flush()

        line = proc.stdout.readline()
        if not line:
            stderr = proc.stderr.read() if proc.stderr else ""
            raise RuntimeError(f"评估进程意外退出。stderr: {stderr[:400]}")

        payload = json.loads(line)
        if not payload.get("ok"):
            message = payload.get("error", "未知错误")
            issues = payload.get("issues") or []
            raise RuntimeError(f"评估失败：{message} {issues}")

        metrics = Metrics.from_json(payload)
        self.cache[key] = metrics
        self.evaluations += 1
        return metrics


# ---------------------------------------------------------------------------
# 采样
# ---------------------------------------------------------------------------


def latin_hypercube(count: int, dim: int, rng: random.Random) -> list[list[float]]:
    """拉丁超立方采样，返回 [0,1]^dim 中的点。

    相比纯随机，它保证每一维都被均匀分层——8 个初始点覆盖 5 维空间时，
    纯随机很容易在某一维上全挤在中间，让 GP 从一开头就缺乏某个方向的
    信息，后续几十次评估都补不回来。
    """
    samples = [[0.0] * dim for _ in range(count)]
    for d in range(dim):
        strata = [(i + rng.random()) / count for i in range(count)]
        rng.shuffle(strata)
        for i in range(count):
            samples[i][d] = strata[i]
    return samples


def denormalize(unit: list[float], names: list[str],
                bounds: dict[str, tuple[float, float]]) -> dict[str, float]:
    out: dict[str, float] = {}
    for value, name in zip(unit, names):
        low, high = bounds[name]
        out[name] = low + (high - low) * min(1.0, max(0.0, value))
    return out


# ---------------------------------------------------------------------------
# 搜索
# ---------------------------------------------------------------------------


@dataclass
class SearchResult:
    budget: int
    names: list[str]
    bounds: dict[str, tuple[float, float]]
    trials: list[Trial] = field(default_factory=list)
    baseline: Trial | None = None

    @property
    def best(self) -> Trial | None:
        scored = [t for t in self.trials if t.phase != "baseline"]
        return max(scored, key=lambda t: t.score) if scored else None

    @property
    def collided_count(self) -> int:
        return sum(1 for t in self.trials if t.metrics.collided)

    def convergence(self) -> list[float]:
        """每次评估后的历史最优，用于画收敛曲线。"""
        out: list[float] = []
        best = -math.inf
        for trial in self.trials:
            if trial.phase == "baseline":
                continue
            best = max(best, trial.score)
            out.append(best)
        return out

    def to_json(self) -> dict[str, Any]:
        best = self.best
        return {
            "budget": self.budget,
            "evaluations": len(self.trials),
            "variables": self.names,
            "bounds": self.bounds,
            "best": None if best is None else {
                "params": best.params,
                "metrics": best.metrics.__dict__,
                "severity": best.score,
            },
            "baseline": None if self.baseline is None else {
                "params": self.baseline.params,
                "metrics": self.baseline.metrics.__dict__,
                "severity": self.baseline.score,
            },
            "collided": self.collided_count,
            "trials": [t.as_row() for t in self.trials],
        }


class RandomSearch:
    """随机搜索基线。

    存在的唯一目的是给贝叶斯优化提供对照：只报"找到了碰撞工况"没有说服力，
    必须说明**在同样预算下随机采样做不到**，否则无法排除"这个问题本来就
    容易，随便采都行"。开题引用 Shahriari 综述声称贝叶斯优于随机，本项目的
    证据就来自这里的同预算对比。
    """

    def __init__(self, evaluator: BatchEvaluator, bounds: dict[str, tuple[float, float]],
                 budget: int = 40, seed: int = 0) -> None:
        self.evaluator = evaluator
        self.bounds = bounds
        self.names = list(bounds)
        self.budget = max(1, budget)
        self.rng = random.Random(seed + 9187)   # 与贝叶斯用不同序列，避免同点

    def run(self, baseline_params: dict[str, float] | None = None) -> SearchResult:
        result = SearchResult(budget=self.budget, names=self.names, bounds=self.bounds)

        if baseline_params is not None:
            metrics = self.evaluator.evaluate(baseline_params)
            result.baseline = Trial(baseline_params, metrics, severity(metrics), "baseline")
            result.trials.append(result.baseline)

        for _ in range(self.budget):
            unit = [self.rng.random() for _ in self.names]
            params = denormalize(unit, self.names, self.bounds)
            metrics = self.evaluator.evaluate(params)
            result.trials.append(Trial(params, metrics, severity(metrics), "random"))

        return result


class BayesianSearch:
    """期望改进驱动的贝叶斯优化。"""

    def __init__(self, evaluator: BatchEvaluator, bounds: dict[str, tuple[float, float]],
                 budget: int = 40, init_points: int = 8, seed: int = 0) -> None:
        self.evaluator = evaluator
        self.bounds = bounds
        self.names = list(bounds)
        self.budget = max(2, budget)
        self.init_points = max(2, min(init_points, self.budget - 1))
        self.rng = random.Random(seed)

    def run(self, baseline_params: dict[str, float] | None = None) -> SearchResult:
        result = SearchResult(budget=self.budget, names=self.names, bounds=self.bounds)

        # 先测基线：模板原值就是"工程师手工调的默认工况"。
        # 没有这个参照，"优化后更危险了"这句话说不清是优化有效还是场景本来就要撞。
        if baseline_params is not None:
            base_metrics = self.evaluator.evaluate(baseline_params)
            result.baseline = Trial(baseline_params, base_metrics,
                                    severity(base_metrics), "baseline")
            result.trials.append(result.baseline)

        # ---- 初始化：拉丁超立方 ----
        units: list[list[float]] = []
        scores: list[float] = []
        for unit in latin_hypercube(self.init_points, len(self.names), self.rng):
            params = denormalize(unit, self.names, self.bounds)
            metrics = self.evaluator.evaluate(params)
            score = severity(metrics)
            result.trials.append(Trial(params, metrics, score, "init"))
            units.append(unit)
            scores.append(score)

        # ---- 主循环 ----
        evaluated = self.init_points
        # 长度尺度只在初始化完成后选一次，之后固定。
        #
        # 每轮都重选会让开销变成 7 倍（候选长度有 7 个，每个都要做一次
        # O(n³) 的 Cholesky）。实测 100 次预算下，每轮重选要 55 s，
        # 选一次只要 8 s 左右。而且数据量小的时候 MLE 本身的估计噪声就大，
        # 反复重选反而让模型在几组超参之间跳来跳去。
        length_scale: float | None = None

        while evaluated < self.budget:
            if length_scale is None:
                gp = fit_best(units, scores)
                length_scale = gp.length_scale
            else:
                gp = GaussianProcess(length_scale=length_scale).fit(units, scores)

            best_score = max(scores) if scores else 0.0

            candidate = self._next_point(gp, best_score, units)
            params = denormalize(candidate, self.names, self.bounds)
            metrics = self.evaluator.evaluate(params)
            score = severity(metrics)

            result.trials.append(Trial(params, metrics, score, "bo"))
            units.append(candidate)
            scores.append(score)
            evaluated += 1

        return result

    def _next_point(self, gp: Any, best: float,
                    evaluated: list[list[float]]) -> list[float]:
        """在"全局随机候选 + 已评估点邻域"两类候选里取 EI 最大的那个。

        只撒全局随机点会浪费预算——最优点附近往往还有更极端的参数组合；
        只做局部扰动又可能卡在局部最优。两类混合是常见的折中做法。

        候选数不能开得太大：EI 需要对每个候选做一次 GP 预测，而预测是
        O(n²) 的矩阵回代。900 个候选、100 个训练点在纯 Python 里已经
        要几秒，再翻倍就得不偿失——多评估几十个候选带来的改进，
        远不如把这几秒换成真实的仿真次数。
        """
        dim = len(self.names)
        candidates: list[list[float]] = []

        for _ in range(700):
            candidates.append([self.rng.random() for _ in range(dim)])

        for seed_point in evaluated[-3:]:
            for _ in range(120):
                point = [
                    min(0.999, max(0.001, v + self.rng.gauss(0.0, 0.08)))
                    for v in seed_point
                ]
                candidates.append(point)

        best_ei = -1.0
        best_point = candidates[0]
        for point in candidates:
            ei = expected_improvement(gp, point, best)
            if ei > best_ei:
                best_ei, best_point = ei, point
        return best_point
