"""自然语言 → 场景模板。两条通路：确定性规则解析、大语言模型解析。

**为什么规则解析器不是"降级方案"，而是基线**

把规则解析当成"没网时的备胎"，就会写得敷衍，最后它只在演示时露一次面。
这里把它当主角之一：

  1. 它是**确定性**的。同一句话解析十次结果一样，可以进单元测试；
     LLM 即使温度设为 0 也不保证跨版本稳定，不能作为回归基线。
  2. 它是**对照**。两条通路在同一句话上给出不同场景，说明这句话本身
     有歧义（"跟得近一点"到底多近？），这种分歧值得记成用例而不是当 bug。
  3. 它**不需要密钥**。毕设评审、答辩现场、离线板卡上都能跑。

因此生成器默认先跑规则解析；只有显式启用 LLM 时才去调用模型，并保留
规则解析结果作为对照与兜底。

**LLM 通路的工程约束**

  * 只走标准库 `urllib`，不引入 SDK——与 `python/` 侧"零依赖"的原则一致。
  * 接口按 OpenAI 兼容的 `/chat/completions` 写，改 `base_url` 即可对接
    国内各家兼容端点，不必为每家写适配器。
  * 提示词里的字段说明从 `schema.describe_for_prompt()` 派生，不另写一份。
"""

from __future__ import annotations

import json
import math
import os
import re
import urllib.error
import urllib.request
from typing import Any

from .schema import describe_for_prompt
from .template import Actor, Parameter, Scenario

# ---------------------------------------------------------------------------
# 中文数字
# ---------------------------------------------------------------------------

_CN_DIGITS = {
    "零": 0, "〇": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4,
    "五": 5, "六": 6, "七": 7, "八": 8, "九": 9,
}


def cn_number(text: str) -> float | None:
    """解析"三"、"十五"、"二十"这类中文数字，也接受阿拉伯数字。

    只覆盖 0-99：场景里需要中文数字的地方只有车道数、时长、车速这类
    小整数，做完整的中文数字解析是过度设计。
    """
    text = text.strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        pass

    if "十" in text:
        head, _, tail = text.partition("十")
        tens = _CN_DIGITS.get(head, 1) if head else 1
        ones = _CN_DIGITS.get(tail, 0) if tail else 0
        return float(tens * 10 + ones)
    if len(text) == 1 and text in _CN_DIGITS:
        return float(_CN_DIGITS[text])
    return None


# ---------------------------------------------------------------------------
# 槽位抽取
# ---------------------------------------------------------------------------

#: 速度表达式的匹配。顺序有讲究：先匹配带单位的，再匹配裸数字，
#: 否则 "30 km/h" 会被裸数字规则抓成 30 m/s。
_SPEED_PATTERNS = (
    (re.compile(r"(\d+(?:\.\d+)?)\s*(?:km/h|kmh|kph|公里每小时|公里/小时|码)"), 1.0 / 3.6),
    (re.compile(r"(\d+(?:\.\d+)?)\s*(?:m/s|米每秒|米/秒|mps)"), 1.0),
)

_DISTANCE_PATTERN = re.compile(r"(\d+(?:\.\d+)?)\s*(?:m\b|米|公尺)")
_DURATION_PATTERN = re.compile(r"(\d+(?:\.\d+)?)\s*(?:s\b|秒)")
_LANE_PATTERN = re.compile(r"([一二两三四五六\d]+)\s*(?:条|个)?\s*车道")

#: "第 N 秒"、"N 秒后" 表示**触发时刻**，不是场景总时长。
#: 若不先把这类片段抠掉，"第 2 秒前车急刹" 会让 duration 被解析成 2 秒，
#: 于是"触发时刻 2.0 不早于时长 2.0"——场景永远不会发生。
_TRIGGER_PATTERN = re.compile(
    r"(?:第\s*(\d+(?:\.\d+)?)\s*(?:s\b|秒))"
    r"|(?:(\d+(?:\.\d+)?)\s*(?:s\b|秒)\s*(?:之?后|时))")


def _strip_trigger_phrases(text: str) -> str:
    return _TRIGGER_PATTERN.sub(" ", text)


def _first_trigger(text: str) -> float | None:
    """抽取触发时刻。返回 None 表示原话没给。"""
    m = _TRIGGER_PATTERN.search(text)
    if not m:
        return None
    raw = m.group(1) or m.group(2)
    return float(raw) if raw else None


def _first_speed(text: str) -> float | None:
    for pattern, scale in _SPEED_PATTERNS:
        m = pattern.search(text)
        if m:
            return float(m.group(1)) * scale
    return None


def _first_group(pattern: re.Pattern[str], text: str) -> float | None:
    m = pattern.search(text)
    if not m:
        return None
    return cn_number(m.group(1))


def _friction_from_text(text: str) -> float | None:
    """由路况描述推附着系数。

    取值参考常见工程估算区间（干沥青 0.8~0.9、湿沥青 0.5~0.6、
    压实雪 0.25~0.35、冰面 0.1~0.2）。这里取区间中值，宁可保守。
    """
    if re.search(r"冰|结冰|冻", text):
        return 0.15
    if re.search(r"雪", text):
        return 0.30
    # "打滑""路很滑"都是湿滑路面的口语说法，只认"湿滑"会漏掉一半。
    if re.search(r"雨|湿|水|滑", text):
        return 0.55
    if re.search(r"干|晴", text):
        return 0.90
    return None


class Slots:
    """从一句话里抽出的字段。None 表示"这句话没说"，交给后续补默认值。"""

    __slots__ = (
        "duration", "ego_speed", "lead_gap", "lead_speed", "trigger",
        "cut_in", "brake", "pedestrian", "static_obstacle",
        "friction", "lane_count", "curvature",
    )

    def __init__(self) -> None:
        for name in self.__slots__:
            setattr(self, name, None)

    def filled(self) -> list[str]:
        return [n for n in self.__slots__ if getattr(self, n) is not None]


def extract_slots(text: str) -> Slots:
    """按关键词与单位表达式抽取槽位。

    抽取顺序无关紧要，但**关键词要具体到能区分意图**：
    "切入"必须比"前方有车"优先，否则一句话里同时提到两者时会被后者覆盖。
    """
    s = Slots()
    lowered = text.lower()
    # 时长要在"抠掉触发时刻"之后的文本上找，否则会把"第 3 秒"当成总时长。
    s.duration = _first_group(_DURATION_PATTERN, _strip_trigger_phrases(lowered))
    s.lane_count = _first_group(_LANE_PATTERN, lowered)
    s.friction = _friction_from_text(text)
    s.trigger = _first_trigger(lowered)

    if re.search(r"弯|曲线|转弯", text):
        s.curvature = 0.01      # 半径 100 m 的缓弯，足够让横向控制显出差别

    # ---- 意图识别 ----
    # 切入类。关键词要覆盖口语的各种说法："并进来"与"并入"是同一个意思，
    # 少列一个就会出现"用户说了切入、系统当成跟车"的错配。
    if re.search(r"切入|加塞|别车|并入|并进|并线|插进|插队|切进|变道过来|强行并", text):
        s.cut_in = True

    # 急刹类
    if re.search(r"急刹|急减速|突然刹|紧急制动|刹停|前车停|急停|踩死", text):
        s.brake = True

    # 行人
    if re.search(r"行人|横穿|骑车人|自行车", text):
        s.pedestrian = True

    # 静止障碍（抛锚车、落物）
    if re.search(r"静止|抛锚|事故车|落物|障碍物|停在", text):
        s.static_obstacle = True

    # ---- 数值槽位 ----
    speeds = [float(m.group(1)) * scale
              for pattern, scale in _SPEED_PATTERNS
              for m in [pattern.search(lowered)] if m]
    if speeds:
        # 一句话里出现多个速度时，按出现顺序分配给自车与前车。
        s.ego_speed = speeds[0]
        if len(speeds) > 1:
            s.lead_speed = speeds[1]

    if s.ego_speed is None:
        if re.search(r"高速|快速路|畅通", text):
            s.ego_speed = 22.0       # 约 80 km/h
        elif re.search(r"慢速|拥堵|低速|蠕行", text):
            s.ego_speed = 4.0

    distances = [float(m.group(1)) for m in _DISTANCE_PATTERN.finditer(lowered)]
    if distances:
        s.lead_gap = distances[0]

    if s.lead_gap is None:
        if re.search(r"跟得?很?近|贴得?很?近|距离很?近|紧跟|贴近|跟近", text):
            s.lead_gap = 8.0
        elif re.search(r"跟得?很?远|距离很?远|前方很远", text):
            s.lead_gap = 45.0

    if s.lead_speed is None and s.ego_speed is not None and not s.static_obstacle:
        if s.cut_in:
            s.lead_speed = max(0.0, s.ego_speed - 1.0)
        else:
            # 急刹场景里前车初始与自车同速：先稳态跟车，再制动。
            # 若给前车一个更低的速度，威胁就变成了"追上一辆慢车"，
            # 而不是"前车突然刹停"。
            s.lead_speed = s.ego_speed

    return s


# ---------------------------------------------------------------------------
# 槽位 → 场景
# ---------------------------------------------------------------------------

#: 槽位 → 场景的组装。若原话只说得出零散的数值，宁可让上层回退到
#: 预置模板并标注待确认字段，也不要凭空补出一个"看着像"的场景。


def _scenario_id(slots: Slots) -> str:
    """按意图生成场景标识。

    不用原文截断：中文抽不出 ASCII，`_slugify` 会得到 `25_3` 这种
    只剩数字的标识，既不可读也无法在实验表里区分场景。
    """
    if slots.pedestrian:
        return "nl_pedestrian"
    if slots.static_obstacle:
        return "nl_static_obstacle"
    if slots.cut_in:
        # 切入车默认就带制动（"别进来同时踩刹车"），因此不再区分
        # cut_in_brake——那个分支永远命中，等于白写。
        return "nl_cut_in"
    if slots.brake:
        return "nl_lead_brake"
    return "nl_follow_lead"


def build_scenario(slots: Slots, text: str, lane_width: float = 3.5) -> Scenario:
    """把槽位组装成一份完整的场景模板。

    补默认值的原则：**能由已有信息推出的才推，推不出就停下**。
    例如给了自车速度与跟车距离，前车速度可以按"稳态跟车"推成相等；
    但如果连速度都没提，就不该凭空造一个 8 m/s——那是假信息。
    """
    scenario = Scenario()
    scenario.name = " ".join(text.split())[:40] or "自然语言场景"
    scenario.id = _scenario_id(slots)

    scenario.lane_width = lane_width
    scenario.lane_count = int(slots.lane_count) if slots.lane_count else 2
    scenario.friction = slots.friction if slots.friction is not None else 0.9
    scenario.curvature = slots.curvature if slots.curvature is not None else 0.0
    scenario.duration = float(slots.duration) if slots.duration else 30.0
    scenario.ego_speed = slots.ego_speed if slots.ego_speed is not None else 8.0

    if slots.pedestrian:
        # 行人从路侧横穿，速度取 1.4 m/s（成年人步速）。
        ped = Actor(type="pedestrian", x=22.0, y=lane_width * 1.6, speed=1.4,
                    theta=-math.pi / 2, behavior="constant")
        scenario.actors.append(ped)
        scenario.duration = min(scenario.duration, 20.0)
        return scenario

    if slots.static_obstacle:
        ahead = float(slots.lead_gap) if slots.lead_gap else 25.0
        broken = Actor(type="car", x=ahead, y=0.0, speed=0.0, behavior="constant")
        scenario.actors.append(broken)
        return scenario

    # 主前车：同车道、正前方。
    lead_gap = float(slots.lead_gap) if slots.lead_gap else 25.0
    lead_speed = slots.lead_speed if slots.lead_speed is not None else scenario.ego_speed
    lead = Actor(type="car", x=lead_gap, y=0.0, speed=float(lead_speed),
                 behavior="car_following")

    if slots.brake:
        # "急刹"必须落到 `brake` 行为上。若只把前车设成静止，表达的是
        # "前方有静止障碍"，与"原本同速跟车、某刻突然制动"是两回事：
        # 前者自车一上路就看见它，后者要到制动发生才构成威胁。
        lead.behavior = "brake"
        lead.brake_accel = 6.0          # 乘用车极限制动近似
        lead.speed_floor = 0.0
        # 第 2 秒开始制动，留出稳态跟车窗口；原话给了时刻就用原话的。
        lead.trigger = slots.trigger if slots.trigger is not None else 2.0
        scenario.duration = min(scenario.duration, 20.0)

    scenario.actors.append(lead)

    if slots.cut_in:
        # 邻道车：横向在相邻车道，纵向略微落后，速度略高于前车。
        trigger = slots.trigger if slots.trigger is not None else 3.0
        neighbor = Actor(
            type="car",
            x=12.0,
            y=lane_width,
            speed=max(lead_speed + 1.5, scenario.ego_speed * 0.9),
            behavior="cut_in",
            trigger=trigger,
            target_y=0.0,
            lane_change_seconds=4.0,
            brake_accel=3.0,
            speed_floor=1.5,
        )
        scenario.actors.insert(0, neighbor)
        # 前车必须挪远：切入车要在前车与自车之间完成变道，
        # 若前车仍在 25 m，切入车会从它身上穿过去（见 validate 的轨迹互穿检查）。
        lead.x = max(lead_gap, 45.0)
        scenario.duration = max(scenario.duration, trigger + 10.0)

    return scenario


def _slugify(text: str) -> str:
    """中文名转 ASCII 标识：保留字母数字，其余折叠为下划线。"""
    ascii_part = re.sub(r"[^a-zA-Z0-9]+", "_", text).strip("_").lower()
    return ascii_part or "nl_scenario"


class RuleParser:
    """确定性解析器。名字带 `Rule` 是为了在日志里一眼分清走的哪条通路。"""

    name = "rule"

    def parse(self, text: str) -> tuple[Scenario, dict[str, Any]]:
        slots = extract_slots(text)
        scenario = build_scenario(slots, text)

        # 自信的判据是"意图明确"或"关键数值给了"，而不是"槽位够多"。
        # 早先用"至少 3 个槽位"，结果"高速公路快速跟车"这种意图清楚的话
        # 被判为信息不足——槽位只填了 2 个，可它一点也不含糊。
        has_intent = bool(slots.cut_in or slots.brake or slots.pedestrian
                          or slots.static_obstacle)
        has_numbers = any(v is not None for v in
                          (slots.ego_speed, slots.lead_gap, slots.duration))
        confident = has_intent or has_numbers

        info = {
            "slots": {k: getattr(slots, k) for k in slots.filled()},
            "slot_count": len(slots.filled()),
            "confident": confident,
        }
        if not confident:
            scenario.unresolved = ["/actors", "/durationSeconds", "/egoSpeed"]
        return scenario, info


# ---------------------------------------------------------------------------
# LLM 通路
# ---------------------------------------------------------------------------

SYSTEM_PROMPT = """你是自动驾驶仿真平台的场景配置助手。把用户的中文描述转成一份场景 JSON。

规则：
1. 只输出 JSON 对象，不要 Markdown 代码块、不要解释文字。
2. 自车固定在原点、航向沿 +x 正方向；所有参与者坐标以自车为参考。
3. 描述里没提到的字段**直接省略**，不要编造。缺字段时程序会补默认值。
4. 若描述自相矛盾或信息不足，输出 {"_insufficient": true, "reason": "..."}。

{fields}

只输出 JSON。"""


def build_prompt(text: str) -> tuple[str, str]:
    """返回 (system, user)。system 里的字段说明来自 Schema，不另写一份。"""
    return SYSTEM_PROMPT.format(fields=describe_for_prompt()), text


class LlmParser:
    """OpenAI 兼容端点的解析器。

    配置全部来自环境变量，不落配置文件：
      ADASIM_LLM_BASE_URL  例如 https://api.deepseek.com/v1
      ADASIM_LLM_API_KEY   密钥
      ADASIM_LLM_MODEL     模型名，例如 deepseek-chat

    为什么用环境变量而不是配置文件：这类密钥不该进版本库，而一旦有了
    配置文件就会有人把它提交上去。环境变量在进程边界上，天然不进 git。
    """

    name = "llm"

    def __init__(self, base_url: str | None = None, api_key: str | None = None,
                 model: str | None = None, timeout: float = 30.0) -> None:
        self.base_url = (base_url or os.environ.get("ADASIM_LLM_BASE_URL", "")).rstrip("/")
        self.api_key = api_key or os.environ.get("ADASIM_LLM_API_KEY", "")
        self.model = model or os.environ.get("ADASIM_LLM_MODEL", "")
        self.timeout = timeout
        self.last_error = ""

    @property
    def available(self) -> bool:
        return bool(self.base_url and self.api_key and self.model)

    def describe(self) -> str:
        if not self.available:
            missing = [n for n, v in (
                ("ADASIM_LLM_BASE_URL", self.base_url),
                ("ADASIM_LLM_API_KEY", self.api_key),
                ("ADASIM_LLM_MODEL", self.model),
            ) if not v]
            return f"未启用（缺少 {', '.join(missing)}）"
        return f"{self.model} @ {self.base_url}"

    def parse(self, text: str, feedback: list[str] | None = None) -> dict[str, Any] | None:
        """调用模型。`feedback` 是上一轮校验失败的原因，用于回填重试。

        返回解析后的 JSON 对象；调用失败或返回不是 JSON 时返回 None，
        由调用方决定回退——本函数不替上层做决策。
        """
        if not self.available:
            self.last_error = "未配置模型端点"
            return None

        system, user = build_prompt(text)
        if feedback:
            user += "\n\n上一轮输出存在以下问题，请修正后重新输出完整 JSON：\n" + \
                    "\n".join(f"- {item}" for item in feedback)

        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": user},
            ],
            "temperature": 0.1,
            "response_format": {"type": "json_object"},
        }
        request = urllib.request.Request(
            f"{self.base_url}/chat/completions",
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.api_key}",
            },
            method="POST",
        )

        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                body = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", "replace")[:200]
            self.last_error = f"HTTP {exc.code}: {detail}"
            return None
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            self.last_error = f"网络错误: {exc}"
            return None
        except json.JSONDecodeError as exc:
            self.last_error = f"响应不是 JSON: {exc}"
            return None

        try:
            content = body["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError):
            self.last_error = f"响应结构异常: {str(body)[:200]}"
            return None

        return _loads_lenient(content, self)


def _loads_lenient(content: str, parser: LlmParser) -> dict[str, Any] | None:
    """容错解析模型输出。

    即使要求了 `response_format=json_object`，仍有模型会在外面裹一层
    Markdown 代码块；也有端点直接忽略该参数。这里先剥代码围栏，
    再退一步截取首尾花括号。
    """
    text = content.strip()
    if text.startswith("```"):
        text = re.sub(r"^```[a-zA-Z]*\n?", "", text)
        text = re.sub(r"\n?```$", "", text.strip())

    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        start, end = text.find("{"), text.rfind("}")
        if start < 0 or end <= start:
            parser.last_error = f"模型输出不含 JSON 对象: {content[:120]}"
            return None
        try:
            parsed = json.loads(text[start:end + 1])
        except json.JSONDecodeError as exc:
            parser.last_error = f"模型输出无法解析: {exc}"
            return None

    if not isinstance(parsed, dict):
        parser.last_error = "模型输出不是 JSON 对象"
        return None
    return parsed
