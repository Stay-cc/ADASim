"""自然语言场景生成器：编排解析、校验、重试与回退。

**为什么这层必须独立存在**

解析（`nlu`）和校验（`validate`）都可以单独运行，但"解析失败了怎么办"
这件事不属于任何一方：解析器不知道校验规则，校验器不负责产出场景。
把它塞进任何一边都会让那一方变成"什么都管"的上帝对象。

**三级降级链，每一级都明确可观测**

    ① LLM 解析   —— 结构错时把错误回填给模型重试一次（模型改得动类型错）
    ② 规则解析   —— 确定性基线，无密钥可用；槽位足够即采信
    ③ 预置模板   —— 用最接近的模板做骨架，叠加能从原话确认的字段

关键点是第 ③ 级**不是"随便给个默认场景"**：若回退到一张无关模板，
用户看到的是"系统假装成功了"，比明确报错更坏。因此这里按槽位距离挑
最像的一张，并把"哪些字段来自原话、哪些是模板带的"逐条标出来
（`Scenario.unresolved` + `notes`），让人一眼看出哪些需要自己确认。

这条链也是"模型不可用时系统仍然可用"的实现——毕设答辩现场没有网络、
没有密钥，规则解析照样能把一句话变成可跑的仿真场景。
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Any

from .nlu import LlmParser, RuleParser, extract_slots
from .schema import Issue, check_structure
from .template import Actor, Scenario
from .validate import check_all, semantic_ok, structure_ok

#: 预置模板目录（相对本文件）。
PRESET_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "presets")


@dataclass
class GenerationResult:
    """生成结果。除了场景本身，还要说清楚"它是怎么来的"。"""

    scenario: Scenario
    route: str                                  # llm | rule | preset
    issues: list[Issue] = field(default_factory=list)
    attempts: int = 0
    notes: list[str] = field(default_factory=list)
    baseline: Scenario | None = None            # 规则解析的对照结果
    preset_name: str = ""

    @property
    def ok(self) -> bool:
        return structure_ok(self.issues) and semantic_ok(self.issues)

    def report(self) -> str:
        lines = [f"通路     : {self.route}"
                 + (f"（模板：{self.preset_name}）" if self.preset_name else ""),
                 f"尝试次数 : {self.attempts}"]
        if self.scenario.unresolved:
            lines.append("待确认字段: " + "、".join(self.scenario.unresolved))
        if self.issues:
            lines.append("校验问题 :")
            lines.extend("  " + str(i) for i in self.issues)
        if self.notes:
            lines.append("说明     :")
            lines.extend("  " + n for n in self.notes)
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# 预置模板
# ---------------------------------------------------------------------------


def load_presets(directory: str = PRESET_DIR) -> list[Scenario]:
    """加载全部预置模板。目录缺失时返回空列表而不是抛异常——
    生成器仍可退到规则解析，不该因为少了个目录就整个不可用。"""
    presets: list[Scenario] = []
    if not os.path.isdir(directory):
        return presets
    for name in sorted(os.listdir(directory)):
        if not name.endswith(".json"):
            continue
        try:
            presets.append(Scenario.load(os.path.join(directory, name)))
        except (OSError, ValueError, KeyError):
            continue        # 单个模板坏掉不该拖垮整套回退机制
    return presets


def _behavior_signature(scenario: Scenario) -> str:
    """参与者行为的"指纹"，用于判断两个场景在意图上是否同类。"""
    kinds = sorted({a.behavior for a in scenario.actors})
    if any(a.type == "pedestrian" for a in scenario.actors):
        kinds.append("pedestrian")
    if any(a.speed <= 0.01 for a in scenario.actors if a.type in ("car", "truck")):
        kinds.append("static")
    return "|".join(kinds)


def preset_distance(scenario: Scenario, preset: Scenario) -> float:
    """两个场景在意图上的距离，越小越像。

    权重是手工调的，但依据可解释：意图（有没有切入/制动）错一次，
    场景就完全不是那个场景了，所以给 10；速度、间距错一点还能用，给 1。
    与其用一套"更优雅"的自动度量，不如把这种领域常识写死在代码里。
    """
    score = 0.0
    if _behavior_signature(scenario) != _behavior_signature(preset):
        score += 10.0

    if scenario.lane_count != preset.lane_count:
        score += 1.0

    ego_a = max(0.5, scenario.ego_speed)
    ego_b = max(0.5, preset.ego_speed)
    score += min(3.0, abs(ego_a - ego_b) / ego_b * 3.0)

    score += min(2.0, abs(scenario.duration - preset.duration) / 30.0)

    return score


def nearest_preset(scenario: Scenario, presets: list[Scenario]) -> tuple[Scenario | None, float]:
    best: Scenario | None = None
    best_score = float("inf")
    for preset in presets:
        score = preset_distance(scenario, preset)
        if score < best_score:
            best, best_score = preset, score
    return best, best_score


def _merge_into_preset(parsed: Scenario, preset: Scenario, text: str) -> tuple[Scenario, list[str]]:
    """以预置模板为骨架，叠加能从原话里确认的字段。

    只覆盖"原话明确说了"的字段；没说的一律保留模板取值，并记入
    `unresolved`。这是"回退"与"编造"的分界线。
    """
    merged = preset.copy()
    merged.id = preset.id
    merged.name = text.strip()[:40] or preset.name
    notes: list[str] = [f"回退到预置模板「{preset.name}」，字段取自模板与原话的并集"]

    slots = extract_slots(text)
    unresolved: list[str] = []

    if slots.ego_speed is not None:
        merged.ego_speed = slots.ego_speed
    else:
        unresolved.append("/egoSpeed")

    if slots.duration is not None:
        merged.duration = float(slots.duration)
    else:
        unresolved.append("/durationSeconds")

    if slots.friction is not None:
        merged.friction = slots.friction
    else:
        unresolved.append("/friction")

    # 间距与触发时刻落到模板里对应的参与者上，但只有"原话说过"才改。
    if slots.lead_gap is not None:
        for actor in merged.actors:
            if actor.behavior in ("car_following", "constant", "brake") and actor.y == 0.0:
                actor.x = float(slots.lead_gap)
                break
    if slots.trigger is not None:
        for actor in merged.actors:
            if actor.behavior in ("cut_in", "brake"):
                actor.trigger = float(slots.trigger)
    merged.unresolved = unresolved
    if unresolved:
        notes.append("以下字段原话未提及，沿用模板取值：" + "、".join(unresolved))
    return merged, notes


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


def generate(text: str, use_llm: bool = False, llm: LlmParser | None = None,
             preset_dir: str = PRESET_DIR, sample_dir: str | None = None) -> GenerationResult:
    """把一句中文描述变成经过校验的场景模板。

    `use_llm` 为假时只走规则解析——这是默认行为，因为模型调用耗时
    数百毫秒且有外部依赖，不该出现在默认路径上。
    """
    text = (text or "").strip()
    if not text:
        scenario = Scenario(name="空描述", id="empty")
        return GenerationResult(
            scenario=scenario, route="rule",
            issues=[Issue("", "描述为空，无法生成场景", "semantic")],
            notes=["请提供一句场景描述，例如：前车 25 米、同速跟车，第 3 秒邻道车切入并刹车"])

    parser = RuleParser()
    rule_scenario, rule_info = parser.parse(text)

    result = GenerationResult(scenario=rule_scenario, route="rule", baseline=rule_scenario)

    # ---- ① LLM 通路（可选）----
    if use_llm:
        llm = llm or LlmParser()
        if not llm.available:
            result.notes.append(f"LLM 未启用：{llm.describe()}")
        else:
            llm_scenario, llm_issues, attempts = _try_llm(text, llm)
            result.attempts = attempts
            if llm_scenario is not None:
                result.scenario = llm_scenario
                result.route = "llm"
                result.issues = [i for i in llm_issues if i.kind == "hint"]
                _note_rule_divergence(result, llm_scenario, rule_scenario)
                _finalize(result)
                return result
            result.issues.extend(llm_issues)
            result.notes.append(f"LLM 通路失败，改用规则解析：{llm.last_error or '校验未通过'}")

    # ---- ② 规则解析 ----
    rule_issues = check_all(rule_scenario)
    result.issues = list(rule_issues)

    if rule_info["confident"] and structure_ok(rule_issues) and semantic_ok(rule_issues):
        _finalize(result)
        return result

    # ---- ③ 预置模板回退 ----
    presets = load_presets(preset_dir)
    if not presets:
        result.notes.append("未找到预置模板，沿用规则解析结果")
        result.scenario.unresolved = rule_scenario.unresolved or ["/actors"]
        _finalize(result)
        return result

    preset, distance = nearest_preset(rule_scenario, presets)
    if preset is None:
        _finalize(result)
        return result

    merged, notes = _merge_into_preset(rule_scenario, preset, text)
    result.scenario = merged
    result.route = "preset"
    result.preset_name = preset.name
    result.notes.extend(notes)
    if not rule_info["confident"]:
        result.notes.append(
            f"原话只解析出 {rule_info['slot_count']} 个可确定字段，不足以定出场景，故回退")
    result.issues = check_all(merged)
    _finalize(result)
    return result


def _try_llm(text: str, llm: LlmParser) -> tuple[Scenario | None, list[Issue], int]:
    """调用模型，结构错时回填重试一次。

    **只有结构错才重试**。语义错（前车在自车后头、两车重叠）通常是模型
    对物理世界的理解出了问题，重试十次还是错，只是白花时间和 token；
    这类错误直接交给下游标注、由人确认。
    """
    feedback: list[str] = []
    last_issues: list[Issue] = []
    try:
        for attempt in range(1, 3):
            raw = llm.parse(text, feedback=feedback or None)
            if raw is None:
                return None, last_issues, attempt
            if raw.get("_insufficient"):
                last_issues = [Issue("", f"模型认为描述不足：{raw.get('reason', '未说明')}", "semantic")]
                return None, last_issues, attempt

            issues = check_all(raw)
            last_issues = issues
            structure_issues = [i for i in issues if i.kind == "structure"]
            if not structure_issues:
                scenario = Scenario.from_json(raw)
                _normalize(scenario)
                return scenario, issues, attempt
            feedback = [f"{i.path}: {i.message}" for i in structure_issues]
        return None, last_issues, 2
    except Exception as exc:                      # noqa: BLE001
        # 解析器的任何意外都不该让整个生成流程崩掉：上层还有规则解析可用。
        llm.last_error = f"解析异常: {exc}"
        return None, last_issues, 1


def _normalize(scenario: Scenario) -> None:
    """修补模型输出里几处常见的"合法但不合理"。"""
    if not scenario.id or not scenario.id.isascii():
        from .nlu import _slugify
        scenario.id = _slugify(scenario.id or scenario.name)
    # 模型写的中文 name 常带引号或换行，截断并去空白。
    scenario.name = " ".join(scenario.name.split())[:40]
    for actor in scenario.actors:
        actor.type = actor.type.lower().strip()
        actor.behavior = actor.behavior.lower().strip()
        if actor.type not in ("car", "truck", "pedestrian", "cyclist"):
            actor.type = "car"
        if actor.behavior not in ("constant", "car_following", "cut_in", "brake"):
            actor.behavior = "constant"


def _note_rule_divergence(result: GenerationResult, llm_scenario: Scenario,
                          rule_scenario: Scenario) -> None:
    """LLM 与规则解析不一致时记一条说明。

    这不是错误——两条通路本就会在小数上不同。但如果**意图**不一致
    （一个判成切入、一个判成跟驰），说明这句话有歧义，值得人看一眼。
    """
    if _behavior_signature(llm_scenario) != _behavior_signature(rule_scenario):
        result.notes.append(
            "规则解析与模型对这句话的理解不同（"
            f"{_behavior_signature(rule_scenario) or '空'} vs "
            f"{_behavior_signature(llm_scenario) or '空'}），原话可能存在歧义")


def _finalize(result: GenerationResult) -> None:
    """统一收尾：把校验问题里"能被自动修"的部分处理掉。

    目前只做一件事：`hint` 级问题不阻断落盘（它们是建议），
    但要确保 `unresolved` 里不会同时出现已解决的字段。
    """
    resolved = set()
    for issue in result.issues:
        if issue.kind == "semantic":
            resolved.add(issue.path)
    result.scenario.unresolved = [u for u in result.scenario.unresolved if u not in resolved]


# ---------------------------------------------------------------------------
# 命令行辅助
# ---------------------------------------------------------------------------


def describe_issue_json(issues: list[Issue]) -> list[dict[str, str]]:
    return [i.as_dict() for i in issues]


def load_scenario(path: str) -> Scenario:
    with open(path, "r", encoding="utf-8") as fh:
        return Scenario.from_json(json.load(fh))
