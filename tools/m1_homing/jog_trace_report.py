"""离线分析20微步单轴点动记录，区分动作合同、反馈与机械验收。

输入为execute_axis_jog_after_approval.ps1的jog_result.json。模块不导入串口、
不连接调试器、不生成运动命令。报告通过不授予下一次动作许可。
"""
from __future__ import annotations

import argparse
import json
import math
import re
import statistics
from pathlib import Path


class TraceError(ValueError):
    """记录结构或数值无法解释；调用者应保留原始记录，不尝试补发动作。"""


def _integer(value, name: str) -> int:
    if type(value) is not int or value < 0:
        raise TraceError(f'{name} must be a nonnegative integer')
    return value


def _angle(value) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise TraceError('encoder angle must be finite')
    return float(value)


def active_step_trend(samples: list, records: list) -> dict:
    """独立用成对斜率检查STEP中段，避开启用前/禁用后位置偏移。

    M114和随后M408并非同步采样：只取间隔不超过100ms、反馈年龄至多25ms、
    EN已使能且计数4..18的样本。输出仅说明电机角趋势，不等于机械限位方向。
    """
    points=[]
    pattern=r'M408 STATE:1 REASON:0 USED:1 EN:ENABLED STEP:[01] JOG:(\d+) SEEK:0 BACKOFF:0 DRIVER:1 DIR_CONFIRMED:0 HOMED:0 ALLOWED:0 RXERR:0 RXOVF:0'
    for sample in samples:
        if sample['phase']!='active' or not 0<=sample['age_ms']<=25:
            continue
        status=next((r for r in records if r['type']=='RX' and r['text'].startswith('M408 ') and r['ms']>=sample['ms']),None)
        match=re.fullmatch(pattern,status['text']) if status else None
        if not match or status['ms']-sample['ms']>100:
            continue
        count=int(match[1])
        if not 4<=count<=18:
            continue
        if points and count==points[-1][0]:
            continue
        if points and count<points[-1][0]:
            return {'supported_encoder_sign':0,'reason':'counter_decreased'}
        points.append((count,_angle(sample['motor_deg'])))
    if len(points)<4 or points[-1][0]-points[0][0]<8:
        return {'supported_encoder_sign':0,'reason':'insufficient_enabled_step_window','points':len(points)}
    pairs=[(b[1]-a[1],b[0]-a[0]) for i,a in enumerate(points) for b in points[i+1:] if b[0]-a[0]>=3]
    slope=statistics.median(delta/count for delta,count in pairs)
    sign=1 if slope>0 else -1
    agreement=sum(delta*sign>=-0.1 for delta,_ in pairs)/len(pairs)
    displacement=points[-1][1]-points[0][1]
    supported=abs(displacement)>=0.35 and displacement*sign>0 and agreement>=0.8
    return {'supported_encoder_sign':sign if supported else 0,'reason':'supported' if supported else 'small_or_contradictory_step_trend',
            'points':len(points),'first_count':points[0][0],'last_count':points[-1][0],
            'window_delta_motor_deg':displacement,'median_pair_slope_deg_per_count':slope,
            'pair_agreement_fraction':agreement,'method':'enabled STEP4..18 independent pairwise median slope'}


def analyze(data: dict) -> dict:
    """按真实收发与采样重算结果，不信任执行脚本自己给出的pass字段。

    单位：ms为主机相对时间；motor_deg为固件展开电机轴角。此函数不换算
    关节角、不根据编码器正负号设置寻限位方向、不判定机械精度。
    """
    if not isinstance(data, dict):
        raise TraceError('expected a record object')
    axis = data.get('axis')
    if type(axis) is not int or axis not in (1, 2, 3):
        raise TraceError('expected axis 1, 2 or 3')
    records, samples = data.get('records'), data.get('samples')
    if not isinstance(records, list) or not records or not isinstance(samples, list) or not samples:
        raise TraceError('missing records or feedback samples')
    for record in records:
        if not isinstance(record, dict) or record.get('type') not in ('TX', 'RX') or not isinstance(record.get('text'), str):
            raise TraceError('malformed serial record')
        _integer(record.get('ms'), 'record time')
    if any(b['ms'] < a['ms'] for a, b in zip(records, records[1:])):
        raise TraceError('record clock moved backward')
    phases = {'pre': [], 'active': [], 'post': []}
    for sample in samples:
        if not isinstance(sample, dict) or sample.get('phase') not in phases:
            raise TraceError('unexpected feedback phase')
        _integer(sample.get('ms'), 'sample time')
        _integer(sample.get('age_ms'), 'feedback age')
        _angle(sample.get('motor_deg'))
        phases[sample['phase']].append(sample)
    if not all(phases.values()) or any(b['ms'] <= a['ms'] for a, b in zip(samples, samples[1:])):
        raise TraceError('missing phases or nonincreasing sample clock')
    phase_order = {'pre': 0, 'active': 1, 'post': 2}
    if any(phase_order[b['phase']] < phase_order[a['phase']] for a, b in zip(samples, samples[1:])):
        raise TraceError('feedback phase order changed')

    tx = [r for r in records if r['type'] == 'TX']
    rx = [r for r in records if r['type'] == 'RX']
    jogs = [r for r in tx if r['text'] in ('JOG D0','JOG D1')]
    allowed = {'M115', 'M408', 'M114', 'M119', 'M120', 'M121', 'M122', 'CONFIG', 'JOG D0', 'JOG D1', 'KEEP'}
    prefix = ('M115 FIRMWARE:M1HomingPrepare PROTOCOL:M1HOME8 ' if axis == 1 else
              f'M115 FIRMWARE:AxisHomingPrepare PROTOCOL:AXISHOME1 AXIS:{axis} ')
    identity = prefix + ('UART:115200 MOTION_COMPILED:1 AUTO_MOTION:0 JOG_STEPS:20 '
                         'JOG_DEADLINE_MS:500 SEEK_STEPS:200 BACKOFF_STEPS:40 '
                         'HOME_DEADLINE_MS:5500 STEP_PERIOD_MS:20')
    identities = [r['text'] for r in rx if r['text'].startswith('M115 ')]
    config = [r['text'] for r in rx if r['text'].startswith('CONFIG ')]
    config_ok = len(config) == 1
    if config_ok:
        match = re.fullmatch(r'CONFIG VALID:1 RESULT:0 REG:FF READ_ERROR:0 CURRENT_REF:INTERNAL IRUN_CODE:8 IHOLD_CODE:4 GCONF:000001C0 CHOPCONF:14000053 IFCNT:(\d+),(\d+)', config[0])
        config_ok = bool(match) and all(0 <= int(n) <= 255 for n in match.groups()) and (int(match[2])-int(match[1])) % 256 == 4
    done_reply = ('M408 STATE:5 REASON:0 USED:1 EN:DISABLED STEP:0 JOG:20 SEEK:0 '
                  'BACKOFF:0 DRIVER:1 DIR_CONFIRMED:0 HOMED:0 ALLOWED:0 RXERR:0 RXOVF:0')
    done = [r for r in rx if r['text'] == done_reply]
    elapsed = done[0]['ms'] - jogs[0]['ms'] if len(jogs) == 1 and done else None
    limits = [r['text'] for r in rx if r['text'].startswith('M119 ')]
    references = [r['text'] for r in rx if r['text'].startswith('M122 ')]
    checks = {
        'exact_axis_and_bounds': bool(identities) and all(s == identity for s in identities),
        'one_jog_only': len(jogs) == 1 and all(r['text'] in allowed for r in tx),
        'declared_direction_matches_command': len(jogs) == 1 and ('jog_direction' not in data or data['jog_direction'] == jogs[0]['text'].split()[1]),
        'driver_config_confirmed': config_ok and sum(r['text'] == 'CONFIG' for r in tx) == 1,
        'completed_disabled': bool(done) and data.get('final_reply') == done_reply,
        'host_completion_within_650_ms': elapsed is not None and 0 <= elapsed <= 650,
        'feedback_fresh': all(s['age_ms'] < 100 for s in samples),
        'three_limits_open': bool(limits) and all(s == 'M119 M1:OPEN M2:OPEN M3:OPEN' for s in limits),
        'reference_unset_after_jog': bool(references) and bool(re.fullmatch(r'M122 LOCAL_REF:0 VALID:0 REASON:1 .* WHOLE_HOMED:0 ALLOWED:0', references[-1])),
        'no_execution_error': data.get('action_issued') is True and data.get('error') is None and
                              all(not r['text'].startswith('error:') or r['text'] == 'error: no live session' for r in rx),
    }
    # 主机记录的活动时段必须确实覆盖一次JOG，不能混用其他会话的样本。
    checks['sample_timing_matches_jog'] = len(jogs) == 1 and bool(done) and phases['pre'][-1]['ms'] < jogs[0]['ms'] <= phases['active'][0]['ms'] <= done[0]['ms'] <= phases['post'][0]['ms']
    state_trace_ok = len(jogs) == 1
    previous_count = 0
    if state_trace_ok:
        for record in rx:
            if record['ms'] < jogs[0]['ms'] or not record['text'].startswith('M408 '):
                continue
            match = re.fullmatch(r'M408 STATE:([15]) REASON:0 USED:1 EN:(ENABLED|DISABLED) STEP:([01]) JOG:(\d+) SEEK:0 BACKOFF:0 DRIVER:1 DIR_CONFIRMED:0 HOMED:0 ALLOWED:0 RXERR:0 RXOVF:0',record['text'])
            if not match or not previous_count <= int(match[4]) <= 20:
                state_trace_ok = False
                break
            if (match[1]=='5' and record['text']!=done_reply) or (match[1]=='1' and match[2]!='ENABLED'):
                state_trace_ok = False
                break
            previous_count = int(match[4])
    checks['state_trace_monotonic_and_no_fault'] = state_trace_ok
    baseline = float(phases['pre'][-1]['motor_deg'])
    observed = phases['active'] + phases['post']
    peak = max((s['motor_deg']-baseline for s in observed), key=abs)
    net = float(samples[-1]['motor_deg'])-baseline
    post_span = max(s['motor_deg'] for s in phases['post'])-min(s['motor_deg'] for s in phases['post'])
    first_delta = float(phases['active'][0]['motor_deg'])-baseline
    clear_movement = 0.35 <= abs(peak) <= 5.0
    initial_opposite = abs(first_delta) >= 0.35 and first_delta*peak < 0
    warnings = []
    if initial_opposite:
        warnings.append('initial_response_opposes_largest_excursion')
    if min(s['motor_deg']-baseline for s in observed) <= -0.35 and max(s['motor_deg']-baseline for s in observed) >= 0.35:
        warnings.append('feedback_excursions_on_both_sides_of_baseline')
    if post_span > 0.2:
        warnings.append('post_disable_variation_over_0_2_motor_deg')
    if not clear_movement:
        warnings.append('feedback_outside_movement_evidence_range')
    return {
        'axis': axis, 'jog_direction': jogs[0]['text'].split()[1] if len(jogs)==1 else None,
        'control_contract_passed': all(checks.values()),
        'movement_evidence_passed': clear_movement and checks['feedback_fresh'],
        'checks': checks, 'completion_ms': elapsed, 'sample_count': len(samples),
        'max_feedback_age_ms': max(s['age_ms'] for s in samples),
        'max_sample_gap_ms': max(b['ms']-a['ms'] for a,b in zip(samples,samples[1:])),
        'baseline_motor_deg': baseline, 'first_active_delta_motor_deg': first_delta,
        'peak_delta_motor_deg': peak, 'final_delta_motor_deg': net,
        'post_span_motor_deg': post_span, 'warnings': warnings,
        'active_step_trend': active_step_trend(samples,records),
        # 机械验收需要其他证据，不能由命令计数或角度符号自动升级。
        'mechanical_direction_verified': False, 'angle_accuracy_verified': False,
        'home_verified': False, 'next_action_authorized': False,
        'other_axes_physical_motion_verified': False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('record', type=Path)
    args = parser.parse_args()
    try:
        report = analyze(json.loads(args.record.read_text(encoding='utf-8-sig')))
    except (TraceError, OSError, json.JSONDecodeError) as error:
        print(json.dumps({'report_valid': False, 'error': str(error)}, ensure_ascii=False))
        return 2
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if report['control_contract_passed'] and report['movement_evidence_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
