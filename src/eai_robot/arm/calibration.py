"""Software calibration shared by both backends; no hardware imports."""
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
MODEL_NUMBER = 1315
DEFAULT_FEEDBACK_SLACK = 3


def feedback_in_range(raw, low, high, slack=DEFAULT_FEEDBACK_SLACK):
    """Small encoder feedback allowance; commanded targets remain strictly bounded."""
    return type(raw) is int and 0 <= raw <= 1023 and low - slack <= raw <= high + slack


@dataclass(frozen=True)
class JointCalibration:
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int

    def validate(self, expected_id):
        if any(type(v) is not int for v in asdict(self).values()):
            raise ValueError("校准字段必须为整数")
        if self.id != expected_id or self.drive_mode not in (0, 1) or self.homing_offset != 0:
            raise ValueError("关节 ID 应按 1..6 排列，drive_mode 为 0/1，SCS215 homing_offset 为 0")
        if not 0 <= self.range_min < self.range_max <= 1023:
            raise ValueError("校准范围必须是 0..1023 内的递增区间；不支持跨反馈边界")

    def bounds(self, margin=0):
        if type(margin) is not int or margin < 0:
            raise ValueError("margin 必须是非负整数")
        low, high = self.range_min + margin, self.range_max - margin
        if low >= high:
            raise ValueError("端点余量过大，校准范围已为空")
        return low, high

    def position(self, ratio, margin=0):
        if not math.isfinite(ratio) or not 0 <= ratio <= 1:
            raise ValueError("每个输入必须是 0..1 的有限数值")
        low, high = self.bounds(margin)
        return round(low + (1 - ratio if self.drive_mode else ratio) * (high - low))

    def ratio(self, position, margin=0):
        low, high = self.bounds(margin)
        value = (position - low) / (high - low)
        return 1 - value if self.drive_mode else value


def validate(calibration):
    if not isinstance(calibration, dict) or set(calibration) != set(JOINTS):
        raise ValueError("校准文件必须包含且仅包含六个指定关节")
    for sid, name in enumerate(JOINTS, 1):
        calibration[name].validate(sid)
    return calibration


def load(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or set(data) != set(JOINTS):
        raise ValueError("校准文件必须包含且仅包含六个指定关节")
    result = {}
    for name in JOINTS:
        item = data[name]
        if not isinstance(item, dict):
            raise ValueError(f"{name} 的校准项不是对象")
        try:
            result[name] = JointCalibration(
                id=item["id"], drive_mode=item.get("drive_mode", 0),
                homing_offset=item.get("homing_offset", 0),
                range_min=item["range_min"], range_max=item["range_max"],
            )
        except KeyError as exc:
            raise ValueError(f"{name} 缺少校准字段 {exc}") from exc
    return validate(result)


def save(path, calibration):
    validate(calibration)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    try:
        temporary.write_text(json.dumps({n: asdict(calibration[n]) for n in JOINTS},
                                       indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def parse_ratios(text):
    values = [float(part) for part in text.replace(",", " ").split()]
    if len(values) != 6 or not all(math.isfinite(v) and 0 <= v <= 1 for v in values):
        raise ValueError("请输入六个 0..1 的有限数值，以空格或逗号分隔")
    return values
