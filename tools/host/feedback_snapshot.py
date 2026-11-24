"""解析M409的一份完整反馈快照；纯数据模块，不打开串口、不发送运动命令。"""
from dataclasses import dataclass
import math
import re

U32_MAX = (1 << 32) - 1


@dataclass(frozen=True)
class FeedbackSnapshot:
    generation: int
    now_ms: int
    frame: str
    present_mask: int
    valid_mask: int
    fresh_mask: int
    max_age_ms: int
    age_ms: tuple[int, int, int]
    read_ms: tuple[int, int, int]
    motor_deg: tuple[float, float, float]
    joint_deg: tuple[float, float, float]

    @property
    def joint_data_fresh(self) -> bool:
        """仅表示本回复中的三轴关节反馈新鲜，不代表HOMED或运动许可。"""
        return self.frame == 'JOINT' and self.fresh_mask == 7


def parse_feedback(line: str) -> FeedbackSnapshot:
    """严格检查字段、三轴长度、有限值和掩码/年龄一致性。

    AGE是固件生成本条回复时的年龄，不是未来时刻仍有效的保证。
    RAW/UNAVAILABLE帧的JOINT字段不能当作已经标定的关节坐标。
    """
    tokens = line.split()
    if not tokens or tokens[0] != 'M409':
        raise ValueError('expected M409')
    fields = {}
    for token in tokens[1:]:
        key, sep, value = token.partition(':')
        if not sep or key in fields or not value:
            raise ValueError('missing or duplicated feedback field')
        fields[key] = value
    expected = {'GENERATION', 'NOW_MS', 'FRAME', 'PRESENT_MASK', 'VALID_MASK',
                'FRESH_MASK', 'MAX_AGE_MS', 'AGE_MS', 'READ_MS', 'MOTOR', 'JOINT'}
    if set(fields) != expected or fields['FRAME'] not in {'JOINT', 'RAW', 'UNAVAILABLE'}:
        raise ValueError('unknown feedback format')

    def integer(value, upper=U32_MAX):
        if not re.fullmatch(r'[0-9]+', value) or len(value) > 10 or int(value) > upper:
            raise ValueError('invalid unsigned feedback field')
        return int(value)

    def triple(value, parse):
        parts = value.split(',')
        if len(parts) != 3:
            raise ValueError('expected three axes')
        return tuple(parse(x) for x in parts)

    def angle(value):
        if len(value) > 32 or not re.fullmatch(r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?', value):
            raise ValueError('invalid angle')
        result = float(value)
        if not math.isfinite(result):
            raise ValueError('non-finite angle')
        return result

    present = integer(fields['PRESENT_MASK'], 7)
    valid = integer(fields['VALID_MASK'], 7)
    fresh = integer(fields['FRESH_MASK'], 7)
    maximum = integer(fields['MAX_AGE_MS'], (1 << 31) - 1)
    ages = triple(fields['AGE_MS'], integer)
    reads = triple(fields['READ_MS'], lambda x: integer(x, (1 << 31) - 1))
    if maximum == 0 or valid & ~present or fresh & ~valid:
        raise ValueError('invalid feedback masks')
    computed = sum(1 << axis for axis in range(3)
                   if valid & (1 << axis) and ages[axis] < maximum)
    if fresh != computed:
        raise ValueError('freshness mask disagrees with ages')
    for axis in range(3):
        if not present & (1 << axis) and (ages[axis] != U32_MAX or reads[axis] != 0):
            raise ValueError('unobserved axis must carry unknown age')
    return FeedbackSnapshot(integer(fields['GENERATION']), integer(fields['NOW_MS']),
                            fields['FRAME'], present, valid, fresh, maximum, ages, reads,
                            triple(fields['MOTOR'], angle), triple(fields['JOINT'], angle))
