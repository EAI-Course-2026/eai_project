"""白名单意图解析：识别文本只允许映射到固定命令，绝不执行任意文本。

契约来自课程交接文档第 9 节（docs/course/source/HANDOFF.md）：
音频输入 -> 语音识别 -> 白名单意图 -> 动作调度 -> 统一控制器。

规则：

1. ``stop`` 永远先判定，保证停止命令能抢占普通动作；即使一句话里同时含停止词和
   其他命令词，也按 ``stop`` 处理。
2. 其余命令按“最长关键词命中”解析；中英文关键词统一去标点、去空白、转小写后比较。
3. 中文关键词允许作为子串出现；纯 ASCII 关键词必须与整句完全相同，避免
   ``open the door`` 之类的整句误命中。
4. 含否定词（不要/不用/不能/别/勿）的文本一律拒绝，只反馈不驱动；停止词已在上一步
   优先处理，因此 ``别动`` 仍是合法的停止命令。
5. 空文本、无法解析或多余文本都只产生反馈，不产生动作计划。

本模块不导入串口、框架或相机，纯函数，可离线回归。
"""
from __future__ import annotations

from dataclasses import dataclass
import unicodedata

#: 允许的命令白名单；顺序即文档顺序，便于审查与测试比对。
COMMANDS = (
    "stand",
    "wave",
    "move_left",
    "move_right",
    "open_gripper",
    "close_gripper",
    "stop",
)

#: 停止命令必须最先判定，可抢占其他动作。
STOP = "stop"

KEYWORDS: dict[str, tuple[str, ...]] = {
    "stop": ("停止", "停下", "停住", "别动", "不要动", "急停", "停", "stop", "halt"),
    "stand": ("站立", "站好", "起立", "归位", "复位", "stand", "home"),
    "wave": ("挥手", "招手", "打个招呼", "打招呼", "摆一摆", "wave"),
    "move_left": ("转到左边", "向左", "往左", "左移", "左边", "moveleft", "move left", "left"),
    "move_right": ("转到右边", "向右", "往右", "右移", "右边", "moveright", "move right", "right"),
    "open_gripper": ("张开夹爪", "打开夹爪", "松开夹爪", "夹爪张开", "张开", "松手", "放开",
                     "opengripper", "open gripper", "open"),
    "close_gripper": ("闭合夹爪", "关闭夹爪", "合上夹爪", "夹爪闭合", "抓住", "抓取", "夹住", "闭合",
                      "closegripper", "close gripper", "grip", "close"),
}

#: 出现任一否定词即拒绝执行本条文本（停止词已优先判定）。
NEGATIONS = ("不要", "不用", "不能", "别", "勿")


@dataclass(frozen=True)
class Intent:
    """一条被接受的命令。``matched`` 是命中的关键词，便于复核解析依据。"""

    command: str
    matched: str
    text: str


@dataclass(frozen=True)
class Outcome:
    """解析结果：``intent`` 为空表示未接受，``reason`` 为反馈文本。"""

    intent: Intent | None
    reason: str

    @property
    def accepted(self) -> bool:
        return self.intent is not None


def normalize(text: str) -> str:
    """去标点、去空白、转小写，只保留字母、数字和汉字。"""
    if not isinstance(text, str):
        raise TypeError("text must be a string")
    kept = [char for char in text.strip().lower()
            if not char.isspace() and not unicodedata.category(char).startswith(("P", "S", "Z", "C"))]
    return "".join(kept)


def _is_ascii_keyword(keyword: str) -> bool:
    return keyword.isascii()


def _matches(keyword: str, normalized: str, *, allow_containment_ascii: bool) -> bool:
    if _is_ascii_keyword(keyword) and not allow_containment_ascii:
        return normalized == keyword
    return keyword in normalized


def _best_match(normalized: str, command: str) -> str | None:
    hits = [keyword for keyword in KEYWORDS[command]
            if _matches(keyword, normalized, allow_containment_ascii=False)]
    if not hits:
        return None
    return max(hits, key=len)


def parse(text: str) -> Outcome:
    """把识别文本解析成白名单命令；不能解析时只返回反馈原因。"""
    normalized = normalize(text)
    if not normalized:
        return Outcome(None, "未识别到有效文本，未执行")
    # 1. 停止优先：允许任意位置出现停止词（含英文），误判只会取消动作，不会产生运动。
    stop_hit = next((keyword for keyword in KEYWORDS[STOP]
                     if _matches(keyword, normalized, allow_containment_ascii=True)), None)
    if stop_hit:
        return Outcome(Intent(STOP, stop_hit, text), "")
    # 2. 否定词：保守拒绝，避免“不要向左”被当成向左执行。
    for negation in NEGATIONS:
        if negation in normalized:
            return Outcome(None, f"检测到否定词“{negation}”，未执行")
    # 3. 其余命令取最长关键词命中。
    best: tuple[int, str, str] | None = None
    for command in COMMANDS:
        if command == STOP:
            continue
        hit = _best_match(normalized, command)
        if hit is None:
            continue
        candidate = (len(hit), command, hit)
        if best is None or candidate[0] > best[0]:
            best = candidate
    if best is None:
        return Outcome(None, "不在命令白名单内，未执行")
    _, command, hit = best
    return Outcome(Intent(command, hit, text), "")


def feedback_text(outcome: Outcome) -> str:
    """给操作者的单行反馈，解析成功时回显命令与命中关键词。"""
    if outcome.intent is None:
        return f"[voice] 忽略：{outcome.reason}"
    return f"[voice] 命令 {outcome.intent.command}（命中“{outcome.intent.matched}”）"
