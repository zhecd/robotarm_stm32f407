"""校验最终ARM ELF的向量与故障停机分支；无串口/SWD操作。"""
import argparse
import hashlib
import json
import re
import struct
import subprocess
from pathlib import Path


def audit(path, bin_dir, locked):
    data = path.read_bytes()
    assert data[:6] == b'\x7fELF\x01\x01', 'Expected ELF32 little endian'
    text = subprocess.check_output([str(bin_dir / 'arm-none-eabi-nm.exe'), str(path)], text=True)
    symbols = {p[2]: int(p[0], 16) for line in text.splitlines()
               if len(p := line.split()) == 3 and p[0] != 'U'}
    dis = subprocess.check_output([str(bin_dir / 'arm-none-eabi-objdump.exe'), '-d', str(path)], text=True)

    def body(name):
        match = re.search(r'^\w+ <' + re.escape(name) + r'>:\n(.*?)(?=^\w+ <|\Z)', dis, re.M | re.S)
        assert match, name
        return match.group(1)

    offset = struct.unpack_from('<I', data, 28)[0]
    size, count = struct.unpack_from('<HH', data, 42)
    segments = []
    vectors = None
    for index in range(count):
        typ, off, _, addr, length, _, _, _ = struct.unpack_from('<8I', data, offset + index * size)
        if typ == 1 and length and 0x08000000 <= addr < 0x08080000:
            segments.append({'address': addr, 'bytes': length})
            if addr == 0x08000000:
                vectors = data[off:off+400]
    assert vectors
    for index, name in [(2, 'NMI_Handler'), (3, 'HardFault_Handler'),
                        (4, 'MemManage_Handler'), (5, 'BusFault_Handler'), (6, 'UsageFault_Handler')]:
        assert struct.unpack_from('<I', vectors, index * 4)[0] & ~1 == symbols[name] & ~1
        assert '<StartupTrial_FailFromException>' in body(name), name
    assert '<StartupTrial_FailFromException>' in body('Error_Handler')
    assert struct.unpack_from('<I', vectors, (16+50)*4)[0] & ~1 == symbols['TIM5_IRQHandler'] & ~1
    assert '<__wrap_HAL_IncTick>' in body('SysTick_Handler')
    # 汇编Reset内部的CopyDataInit等标签也会被objdump当作段落标题。
    reset = dis.split('<Reset_Handler>:', 1)[1].split('<Default_Handler>:', 1)[0]
    assert '<__wrap_main>' in reset
    for name in ['__wrap_AppHardware_ReadLine', '__wrap_AppSerial_SetMode',
                 '__wrap_Dev_Joint_Step', '__wrap_HAL_TIM_PWM_Start', '__wrap_AppHardware_StartStepTimer']:
        assert name in symbols, name
    if locked:
        assert 'HAL_TIM_PWM_Start' not in symbols and 'Dev_Joint_Step' not in symbols
    else:
        assert 'AppTrial_ReleasePreparedStartup' in symbols
    return {'path': str(path), 'sha256': hashlib.sha256(data).hexdigest(),
            'load_bytes': sum(s['bytes'] for s in segments), 'segments': segments,
            'fault_vectors_and_explicit_cut_branches': True, 'TIM5_vector': True,
            'SysTick_and_main_wrapping': True, 'compile_locked': locked, 'flashed': False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=Path)
    parser.add_argument('--bin-dir', required=True, type=Path)
    parser.add_argument('--locked', action='store_true')
    args = parser.parse_args()
    print(json.dumps(audit(args.elf, args.bin_dir, args.locked), indent=2))
