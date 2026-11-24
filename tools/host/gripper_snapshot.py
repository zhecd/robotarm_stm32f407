"""M410夹爪软件记录；纯解析，不连接串口，不声称机械到位。"""
from dataclasses import dataclass
import math
import re


@dataclass(frozen=True)
class GripperSnapshot:
    bound: bool
    target: str
    target_angle_deg: float
    pwm_requested: bool
    hold_ms: int
    elapsed_ms: int
    error: int

    @property
    def hold_finished(self) -> bool:
        """仅表示软件记录的保持窗口结束；不能推断夹物成功或任务许可。"""
        return (self.bound and self.target != 'NONE' and self.error == 0
                and not self.pwm_requested and self.elapsed_ms == self.hold_ms)


def parse_gripper(line: str) -> GripperSnapshot:
    """严格解析完整字段；未绑定、错误、未确认停止均不伪造完成。"""
    tokens = line.split()
    if not tokens or tokens[0] != 'M410':
        raise ValueError('expected M410')
    fields = {}
    for token in tokens[1:]:
        key, sep, value = token.partition(':')
        if not sep or not value or key in fields:
            raise ValueError('missing or duplicated gripper field')
        fields[key] = value
    expected = {'BOUND', 'TARGET', 'ANGLE_DEG', 'PWM_REQUESTED', 'HOLD_MS',
                'ELAPSED_MS', 'ERROR', 'POSITION'}
    if set(fields) != expected or fields['POSITION'] != 'UNMEASURED':
        raise ValueError('unknown gripper format or unverified position claim')

    def integer(key, upper):
        value = fields[key]
        if not re.fullmatch(r'[0-9]{1,10}', value) or int(value) > upper:
            raise ValueError('invalid gripper integer: ' + key)
        return int(value)

    bound = bool(integer('BOUND', 1))
    active = bool(integer('PWM_REQUESTED', 1))
    hold = integer('HOLD_MS', 600)
    elapsed = integer('ELAPSED_MS', 600)
    error = integer('ERROR', 2)
    target = fields['TARGET']
    value = fields['ANGLE_DEG']
    if (target not in {'NONE', 'OPEN', 'CLOSE', 'CUSTOM'} or len(value) > 32
            or not re.fullmatch(r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?', value)):
        raise ValueError('invalid target or angle')
    angle = float(value)
    if not math.isfinite(angle) or not 0 <= angle <= 180 or hold != 600:
        raise ValueError('invalid angle or changed hold contract')
    expected_angle = {'NONE': 0.0, 'OPEN': 100.0, 'CLOSE': 72.0}.get(target)
    if expected_angle is not None and angle != expected_angle:
        raise ValueError('target and angle disagree')
    if not bound and (target != 'NONE' or active or elapsed or error):
        raise ValueError('unbound gripper carries active state')
    if target == 'NONE' and active and error == 0:
        raise ValueError('active gripper without accepted target or error')
    if target == 'NONE' and error == 0 and elapsed:
        raise ValueError('elapsed window without accepted target')
    return GripperSnapshot(bound, target, angle, active, hold, elapsed, error)
