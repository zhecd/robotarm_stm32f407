"""解析IRA1分轴附加快照；必须提供读记录前后的两份相同偶数序列头。"""
import argparse
import json
import struct
from pathlib import Path
from parse_irq_evidence import decode_record_words

HEADER_BYTES, RECORD_BYTES, CAPACITY = 60, 84, 64


def decode_axes(headers_before, records, headers_after):
    if len(headers_before) != 3*HEADER_BYTES or len(headers_after) != 3*HEADER_BYTES or len(records) != 3*CAPACITY*RECORD_BYTES:
        raise ValueError('Wrong IRA1 dump length')
    if headers_before != headers_after:
        raise ValueError('Axis header changed during read; retry snapshot without resetting')
    axes = []
    for axis in range(3):
        h = struct.unpack_from('<15I', headers_before, axis*HEADER_BYTES)
        magic, ident, headcap, tailcap, size, head, tail, nxt, total, overwritten, frozen, imr, freezevec, ignored, revision = h
        if (magic, ident, headcap, tailcap, size) != (0x49524131, axis, 32, 32, 84):
            raise ValueError('Wrong IRA1 layout/axis')
        if revision & 1:
            raise ValueError('Axis writer is in progress; retry snapshot')
        if head > 32 or tail > 32 or nxt >= 32 or frozen > 1 or ignored > total:
            raise ValueError('Invalid axis counters')
        if tail and head != 32 or not frozen and (imr or freezevec or ignored):
            raise ValueError('Invalid ring/freeze state')
        if frozen and (imr & (1 << (10+axis)) or not freezevec):
            raise ValueError('Freeze does not describe a masked axis')
        # 计数饱和时不强行推导精确累计算术；仍校验ABI、序列锁、轴和有效槽。
        stored_events = total-ignored
        saturated = total == 0xffffffff or ignored == 0xffffffff or overwritten == 0xffffffff
        if not saturated:
            if head != min(32, stored_events) or tail != min(32, max(0, stored_events-32)):
                raise ValueError('Published counts do not match events')
            if overwritten != max(0, stored_events-64) or nxt != max(0, stored_events-32) % 32:
                raise ValueError('Invalid tail chronology')
        slots = list(range(head))
        slots += [32+(nxt+i) % 32 for i in range(tail)] if tail == 32 else list(range(32,32+tail))
        parsed = []
        for index, slot in enumerate(slots):
            w = struct.unpack_from('<21I', records, (axis*64+slot)*84)
            vector, event = w[:2]
            r = decode_record_words(w[2:], index)
            if not vector or not event or not r['callback_mask'] & (1 << axis):
                raise ValueError('Record does not belong to selected axis')
            if not saturated:
                expected = slot+1 if slot < 32 else stored_events-tail+(index-head)+1
                if event != expected:
                    raise ValueError('Axis event sequence is inconsistent')
            if parsed and (vector < parsed[-1]['vector_sequence'] or event < parsed[-1]['axis_sequence']):
                raise ValueError('Nonchronological vector/event sequence')
            r.update(physical_slot=slot, vector_sequence=vector, axis_sequence=event)
            parsed.append(r)
        if frozen and (not parsed or parsed[-1]['vector_sequence'] != freezevec):
            raise ValueError('Frozen tail does not end at freeze vector')
        axes.append({'axis': axis+1, 'events_total': total, 'head_count': head, 'tail_count': tail,
                     'tail_overwrites': overwritten, 'frozen': bool(frozen), 'freeze_imr': imr,
                     'freeze_vector': freezevec, 'ignored_after_freeze': ignored, 'revision': revision,
                     'counters_saturated': saturated, 'records': parsed})
    return {'format': 'IRA1', 'stable_headers_verified': True, 'axes': axes,
            'physical_waveform_measured': False, 'root_cause_determined': False,
            'cycles_elapsed_includes_axis_publication': False}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('headers_before', type=Path)
    p.add_argument('records', type=Path)
    p.add_argument('headers_after', type=Path)
    a = p.parse_args()
    print(json.dumps(decode_axes(a.headers_before.read_bytes(), a.records.read_bytes(), a.headers_after.read_bytes()), indent=2))
