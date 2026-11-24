"""离线解析IRQ2快照；分类是寄存器事实，不自动认定电气或软件根因。"""
import argparse
import json
import struct
from pathlib import Path


def decode_record_words(w, index):
    """IRQ2与分轴附加记录共用的19字字段解释；不作根因推断。"""
    before, after, callback = w[11:14], w[14:17], w[17]
    return {'index': index, 'tick_ms': w[0], 'cycles_enter': w[1],
            'cycles_elapsed': (w[18]-w[1]) & 0xffffffff,
            'pending_enter': w[2], 'mask_enter': w[3], 'gpio_c': w[4],
            'active_limit_mask': (~w[4] >> 10) & 7,
            'mapped_to_PC10_PC11_PC12': ((w[5] >> 8) & 255) == 0x22 and (w[6] & 15) == 2,
            'rising': w[7], 'falling': w[8], 'software_trigger': w[9], 'active_vector': w[10],
            'before': before, 'after': after, 'callback_mask': callback,
            'callback_without_pending_at_call': [a+1 for a in range(3) if callback & (1 << a) and not before[a] & (1 << (10+a))],
            'pending_after_handler': [a+1 for a in range(3) if callback & (1 << a) and after[a] & (1 << (10+a))]}


def decode(header_bytes, record_bytes):
    if len(header_bytes) != 36 or len(record_bytes) != 64 * 76:
        raise ValueError('Wrong IRQ2 dump length')
    h = struct.unpack('<9I', header_bytes)
    if h[:3] != (0x49525132, 64, 76) or h[3] > 64 or h[4] < h[3]:
        raise ValueError('Wrong IRQ2 header or publication count')
    records = []
    for index in range(h[3]):
        w = struct.unpack_from('<19I', record_bytes, index * 76)
        records.append(decode_record_words(w, index))
    return {'published': h[3], 'vectors_total': h[4], 'dropped': h[5], 'callback_totals': h[6:9],
            'records': records, 'physical_waveform_measured': False, 'root_cause_determined': False}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('header', type=Path)
    p.add_argument('records', type=Path)
    a = p.parse_args()
    print(json.dumps(decode(a.header.read_bytes(), a.records.read_bytes()), indent=2))
