"""视觉任务前的只读检查：整合软件快照，不打开串口，也不产生动作许可。"""
from dataclasses import dataclass
import math
import re

from feedback_snapshot import FeedbackSnapshot
from gripper_snapshot import GripperSnapshot


def _fields(line, prefix):
    """拒绝缺值、重复字段和格式漂移，避免把不完整回复当作正常。"""
    tokens = line.split()
    if not tokens or tokens[0] != prefix:
        raise ValueError('expected ' + prefix)
    fields = {}
    for token in tokens[1:]:
        key, sep, value = token.partition(':')
        if not sep or not value or key in fields:
            raise ValueError('missing or duplicated field: ' + prefix)
        fields[key] = value
    return fields


def _unsigned(value, upper=(1 << 32) - 1):
    if not re.fullmatch(r'[0-9]{1,10}', value) or int(value) > upper:
        raise ValueError('invalid unsigned integer')
    return int(value)


@dataclass(frozen=True)
class RuntimeSnapshot:
    state: str
    homed: bool
    allowed: bool
    mode: str
    fault: int
    motion_fault: int
    queue: int
    pending: int
    recovery: bool
    irq_latch_mask: int


def parse_runtime(line):
    """正常App和AUDIT共用的M408；诊断/波形的不同结构不混用。"""
    f = _fields(line, 'M408')
    if set(f) != {'STATE', 'HOMED', 'ALLOWED', 'MODE', 'FAULT', 'MOTION_FAULT',
                  'QUEUE', 'PENDING', 'RECOVERY', 'IRQ_LATCH_MASK'}:
        raise ValueError('unsupported runtime status format')
    # 保留未知状态名也不会放行：评估器只接受精确的READY/GCODE。
    return RuntimeSnapshot(f['STATE'], bool(_unsigned(f['HOMED'], 1)),
                           bool(_unsigned(f['ALLOWED'], 1)), f['MODE'],
                           _unsigned(f['FAULT']), _unsigned(f['MOTION_FAULT']),
                           _unsigned(f['QUEUE']), _unsigned(f['PENDING']),
                           bool(_unsigned(f['RECOVERY'], 1)),
                           _unsigned(f['IRQ_LATCH_MASK'], 7))


def parse_limits(line):
    """M119转换成逻辑轴位图，M1/M2/M3分别占bit0/1/2。"""
    f = _fields(line, 'M119')
    if set(f) != {'M1', 'M2', 'M3'} or any(v not in {'OPEN', 'TRIGGERED'} for v in f.values()):
        raise ValueError('invalid limit status')
    return sum(1 << axis for axis in range(3) if f[f'M{axis + 1}'] == 'TRIGGERED')


def supported_identity(line):
    """只有明确的正常/AUDIT协议才继续读取本模块理解的状态格式。"""
    f = _fields(line, 'M115')
    normal = (f.get('FIRMWARE') == 'RobotArm' and f.get('PROTOCOL') == '1'
              and f.get('MOTOR_ENABLE') != 'LOCKED')
    audit = f.get('FIRMWARE') == 'RobotArmStartupAudit' and f.get('PROTOCOL') == 'AUDIT1'
    features = set(f.get('FEATURES', '').split(','))
    return (normal or audit) and f.get('UART') == '115200' and f.get('AXES') == '3' and {
        'FEEDBACK_AGE', 'GRIPPER_STATUS'}.issubset(features), normal


@dataclass(frozen=True)
class PreflightReport:
    """一次观测结果。通过只说明本轮数据满足条件，不是运动授权或实时监控。"""
    identity: str
    started_at: float
    finished_at: float
    blockers: tuple[str, ...]
    before: RuntimeSnapshot | None = None
    after: RuntimeSnapshot | None = None
    limit_mask: int | None = None
    feedback: FeedbackSnapshot | None = None
    gripper: GripperSnapshot | None = None

    @property
    def observations_ready(self):
        return not self.blockers


def evaluate_observations(identity, before, after, limit_mask, feedback, gripper,
                          *, uncertain, started_at, finished_at, feedback_started_at,
                          max_duration_s=2.0):
    """纯评估，输入来自串行查询，不能声称各字段是同一时刻的原子快照。

    反馈年龄加上从M409发送前到评估结束的主机耗时，保守计入传输和后续查询。
    使用主机单调时钟；不把MCU的NOW_MS和电脑时钟相减。
    """
    times = (started_at, finished_at, feedback_started_at, max_duration_s)
    if (any(not math.isfinite(t) for t in times) or max_duration_s <= 0
            or not started_at <= feedback_started_at <= finished_at):
        raise ValueError('invalid monotonic observation window')
    supported, normal = supported_identity(identity)
    blockers = []
    if not supported: blockers.append('UNSUPPORTED_FIRMWARE')
    if not normal: blockers.append('OUTPUT_LOCKED_FIRMWARE')
    if uncertain: blockers.append('COMMUNICATION_UNCERTAIN')
    if finished_at - started_at >= max_duration_s: blockers.append('OBSERVATION_WINDOW_EXPIRED')
    if before != after: blockers.append('RUNTIME_CHANGED')
    if any(s.state != 'READY' or not s.homed or not s.allowed for s in (before, after)):
        blockers.append('NOT_HOMED_READY')
    if any(s.mode != 'GCODE' for s in (before, after)): blockers.append('NOT_GCODE_MODE')
    if any(s.fault or s.motion_fault or s.irq_latch_mask for s in (before, after)):
        blockers.append('FAULT_OR_IRQ_LATCH')
    if any(s.queue or s.pending or s.recovery for s in (before, after)):
        blockers.append('BUSY_OR_RECOVERING')
    if limit_mask: blockers.append('LIMIT_TRIGGERED')
    elapsed_ms = (finished_at - feedback_started_at) * 1000.0
    if (not feedback.joint_data_fresh
            or any(age + elapsed_ms >= feedback.max_age_ms for age in feedback.age_ms)):
        blockers.append('JOINT_FEEDBACK_NOT_FRESH')
    if not gripper.hold_finished:
        blockers.append('GRIPPER_UNAVAILABLE_OR_ACTIVE')
    return PreflightReport(identity, started_at, finished_at, tuple(blockers),
                           before, after, limit_mask, feedback, gripper)
