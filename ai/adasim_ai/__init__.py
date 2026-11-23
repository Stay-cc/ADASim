"""ADASim 的 AI 增强模块。

两个子功能共用"参数化场景模板"这一份描述：

  * `generate` —— 自然语言 → 场景模板（`nlu` 解析、`validate` 校验）；
  * `search`   —— 在模板的参数空间内做贝叶斯失效搜索（见 `search.py`）。

二者与 C++ 底座之间只有"场景文件"与"指标"两个契约，不共享内存、不共享
进程。这条边界是刻意的：AI 模块可能耗时数百毫秒甚至数秒，绝不能出现在
实时仿真循环里。
"""

from .schema import Issue, check_structure
from .template import Actor, Parameter, Scenario, Sensor
from .validate import check_all, check_semantics, semantic_ok, structure_ok

__all__ = [
    "Actor",
    "Issue",
    "Parameter",
    "Scenario",
    "Sensor",
    "check_all",
    "check_semantics",
    "check_structure",
    "semantic_ok",
    "structure_ok",
]

__version__ = "1.0.0"
