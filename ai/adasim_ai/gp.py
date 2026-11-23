"""高斯过程回归（零依赖实现）。

失效搜索要在一个"不可导、带噪声、每次评估要跑一遍仿真"的目标函数上
找极值。贝叶斯优化是这类问题的标准解，而它的代理模型就是高斯过程。

**为什么自己写而不装 scikit-learn**

与 `python/` 侧同样的理由：毕设环境不该因为装不上第三方库就跑不起来。
但这不只是"能用就行"——本模块顺带把 GP 的两处关键数值细节写明白了：

  * 用 Cholesky 分解而不是直接求逆：`K⁻¹` 的条件数会平方级恶化，
    n=100 时直接求逆已经能看出明显的数值漂移；
  * 预测方差出现 -1e-14 这类负数时夹到 0：这是浮点误差，不是物理，
    若留着它，期望改进里的 `sqrt(σ²)` 会变成 NaN，采集函数整片失效。

核函数选 Matérn 5/2 而不是 RBF：失效边界通常是"折线"而不是光滑曲面
（间距略大就安全、略小就撞），RBF 假设的二阶可导在边界附近会过度平滑，
把真正的拐点抹掉。Matérn 5/2 只假设二阶均方可导，对拐点宽容得多。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

#: Matérn 5/2 的常数。写成具名常量，公式里就不必反复出现 5.0 与 3.0。
_SQRT5 = math.sqrt(5.0)


def _cholesky(matrix: list[list[float]]) -> list[list[float]] | None:
    """下三角 Cholesky 分解 A = L·Lᵀ。矩阵非正定时返回 None。

    非正定在 GP 里不是罕见的异常，而是"噪声方差给小了"的常规症状：
    两个几乎相同的输入点各自带不同观测值时，核矩阵会接近奇异。
    返回 None 让调用方去加抖动重试，比抛异常更便于自动处理。
    """
    n = len(matrix)
    lower = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            acc = matrix[i][j]
            for k in range(j):
                acc -= lower[i][k] * lower[j][k]
            if i == j:
                if acc <= 1e-12:
                    return None
                lower[i][j] = math.sqrt(acc)
            else:
                lower[i][j] = acc / lower[j][j]
    return lower


def _solve_lower(lower: list[list[float]], rhs: list[float]) -> list[float]:
    """前代：解 L·x = b。"""
    n = len(lower)
    out = [0.0] * n
    for i in range(n):
        acc = rhs[i]
        for k in range(i):
            acc -= lower[i][k] * out[k]
        out[i] = acc / lower[i][i]
    return out


def _solve_upper_transposed(lower: list[list[float]], rhs: list[float]) -> list[float]:
    """回代：解 Lᵀ·x = b。"""
    n = len(lower)
    out = [0.0] * n
    for i in range(n - 1, -1, -1):
        acc = rhs[i]
        for k in range(i + 1, n):
            acc -= lower[k][i] * out[k]
        out[i] = acc / lower[i][i]
    return out


def matern52(a: list[float], b: list[float], length_scale: float) -> float:
    """Matérn 5/2 核（信号方差归一化为 1，由调用方另乘）。"""
    dist = math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))
    scaled = _SQRT5 * dist / max(1e-9, length_scale)
    return (1.0 + scaled + scaled * scaled / 3.0) * math.exp(-scaled)


@dataclass
class GaussianProcess:
    """标量输出的 GP 回归器。

    输入必须已归一化到 [0,1]^d：长度尺度是"归一化空间里的距离"，
    若各维量纲不同（速度 5~16 m/s、噪声 0~1 m），同一个长度尺度
    对它们意味着完全不同的实际跨度。
    """

    length_scale: float = 0.3
    signal_var: float = 1.0
    noise_var: float = 1e-6
    _x: list[list[float]] = field(default_factory=list)
    _y: list[float] = field(default_factory=list)
    _alpha: list[float] = field(default_factory=list)
    _lower: list[list[float]] = field(default_factory=list)
    _mean: float = 0.0
    _fitted: bool = False

    # ---- 训练 ----

    def fit(self, xs: list[list[float]], ys: list[float]) -> "GaussianProcess":
        self._x = [list(x) for x in xs]
        self._y = list(ys)
        self._fitted = False
        n = len(xs)
        if n == 0:
            return self

        # 去均值后再拟合：目标（失效严重度）是 (0, 2] 的正数，均值远大于 0，
        # 不减掉会让先验"所有点都接近均值"，远离样本处预测不出差异。
        self._mean = sum(ys) / n
        centered = [y - self._mean for y in ys]

        kernel = [[self.signal_var * matern52(xs[i], xs[j], self.length_scale)
                   for j in range(n)] for i in range(n)]

        # 抖动逐次放大：两点几乎重合时核矩阵会接近奇异，
        # 一次性给个大抖动会让模型对真实函数过度平滑。
        for jitter in (0.0, 1e-10, 1e-8, 1e-6, 1e-4):
            candidate = [row[:] for row in kernel]
            for i in range(n):
                candidate[i][i] += self.noise_var + jitter
            lower = _cholesky(candidate)
            if lower is not None:
                self._lower = lower
                self._alpha = _solve_upper_transposed(lower, _solve_lower(lower, centered))
                self._fitted = True
                break

        return self

    # ---- 预测 ----

    def predict(self, x: list[float]) -> tuple[float, float]:
        """返回 (均值, 方差)。未拟合时退化为先验（均值 0、方差为信号方差）。"""
        if not self._fitted or not self._x:
            return self._mean, self.signal_var

        k = [self.signal_var * matern52(x, xi, self.length_scale) for xi in self._x]
        mean = self._mean + sum(ki * ai for ki, ai in zip(k, self._alpha))
        v = _solve_lower(self._lower, k)
        variance = self.signal_var - sum(vi * vi for vi in v)
        # 浮点误差会让远离样本的方差算成 -1e-14，夹到极小正数。
        return mean, max(variance, 1e-12)

    # ---- 超参数选择 ----

    def log_marginal_likelihood(self) -> float:
        """对数边际似然，用于在候选长度尺度里挑最合适的那个。"""
        if not self._fitted or not self._y:
            return -math.inf
        n = len(self._y)
        centered = [y - self._mean for y in self._y]
        quad = sum(a * y for a, y in zip(self._alpha, centered))
        log_det = 2.0 * sum(math.log(self._lower[i][i]) for i in range(n))
        return -0.5 * quad - 0.5 * log_det - 0.5 * n * math.log(2.0 * math.pi)


_LENGTH_CANDIDATES = (0.1, 0.15, 0.2, 0.3, 0.45, 0.65, 0.9)


def fit_best(xs: list[list[float]], ys: list[float],
             noise_var: float = 1e-6) -> GaussianProcess:
    """在候选长度尺度上按边际似然选一个，返回拟合好的 GP。

    不做梯度优化：候选集只有 7 个，网格搜索的代价可以忽略，而手写的
    梯度下降在有噪声的目标上容易收敛到边界。简单方法在这里更可靠，
    也更容易在论文里解释清楚。
    """
    best: GaussianProcess | None = None
    best_score = -math.inf
    for length in _LENGTH_CANDIDATES:
        gp = GaussianProcess(length_scale=length, noise_var=noise_var).fit(xs, ys)
        score = gp.log_marginal_likelihood()
        if score > best_score:
            best, best_score = gp, score
    return best or GaussianProcess(noise_var=noise_var).fit(xs, ys)


# ---------------------------------------------------------------------------
# 采集函数
# ---------------------------------------------------------------------------


def normal_pdf(z: float) -> float:
    return math.exp(-0.5 * z * z) / math.sqrt(2.0 * math.pi)


def normal_cdf(z: float) -> float:
    return 0.5 * (1.0 + math.erf(z / math.sqrt(2.0)))


def expected_improvement(gp: GaussianProcess, x: list[float],
                         best: float, xi: float = 0.01) -> float:
    """期望改进（对最大化问题）。

    EI 相比"直接取预测均值最大"的价值在于它同时考虑不确定性：
    均值略低但方差大的点仍可能藏着更好的解，这一点正是贝叶斯优化
    区别于代理模型贪心搜索的地方。
    """
    mean, variance = gp.predict(x)
    sigma = math.sqrt(max(variance, 1e-12))
    improvement = mean - best - xi
    z = improvement / sigma
    return improvement * normal_cdf(z) + sigma * normal_pdf(z)
