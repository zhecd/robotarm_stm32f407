"""从准确ELF符号生成只读SWD取证配置；仅写本机文件，不连接/复位板卡。"""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

EXPECTED = {'g_irq_axis_evidence':180, 'g_irq_axis_records':16128,
            'g_irq_evidence':36, 'g_irq_records':4864}


def make_cfg(symbols, prefix):
    prefix = Path(prefix).resolve()
    if any(c in prefix.as_posix() for c in '{}\r\n'):
        raise ValueError('Unsafe Tcl output path')
    for name, size in EXPECTED.items():
        if name not in symbols or symbols[name]['bytes'] != size:
            raise ValueError('ELF lacks expected axis evidence ABI: '+name)
        address = symbols[name]['address']
        if address < 0x20000000 or address+size > 0x20020000:
            raise ValueError('Evidence symbol outside SRAM: '+name)
    s = '''source [find interface/stlink.cfg]
transport select dapdirect_swd
source [find target/stm32f4x.cfg]
reset_config none
cortex_m reset_config sysresetreq
adapter speed 1000
stm32f4x.cpu configure -event examine-end {}
bindto 127.0.0.1
gdb_port disabled
telnet_port disabled
tcl_port disabled
init
set en [lindex [read_memory 0x40020c10 32 1] 0]
set a [lindex [read_memory 0x40020010 32 1] 0]
set b [lindex [read_memory 0x40020410 32 1] 0]
set pwm [lindex [read_memory 0x40000000 32 1] 0]
set ccer [lindex [read_memory 0x40000020 32 1] 0]
set step_timer [lindex [read_memory 0x40001000 32 1] 0]
if {($en&4)==0 || ($a&0xc2)!=0 || ($b&1)!=0 || ($pwm&1)!=0 || ($ccer&0x1111)!=0 || ($step_timer&1)!=0} {error "OUTPUTS_NOT_DISABLED"}
'''
    for axis in range(3):
        addr=symbols['g_irq_axis_evidence']['address']+axis*60
        s+=f'set h [read_memory 0x{addr:08x} 32 2]\n'
        s+=f'if {{[lindex $h 0]!=0x49524131 || [lindex $h 1]!={axis}}} {{error "WRONG_AXIS_ABI"}}\n'
    for symbol, suffix in [('g_irq_axis_evidence','axis_headers_before'),
                           ('g_irq_axis_records','axis_records'),
                           ('g_irq_axis_evidence','axis_headers_after'),
                           ('g_irq_evidence','irq_header'),('g_irq_records','irq_records')]:
        path = prefix.with_name(prefix.name+'_'+suffix+'.bin')
        item = symbols[symbol]
        s += f'dump_image {{{path.as_posix()}}} 0x{item["address"]:08x} {item["bytes"]}\n'
    return s+'poll\nshutdown\n'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('elf',type=Path)
    p.add_argument('prefix',type=Path)
    p.add_argument('--bin-dir',required=True,type=Path)
    a=p.parse_args()
    text=subprocess.check_output([str(a.bin_dir/'arm-none-eabi-nm.exe'),'-S',str(a.elf)],text=True)
    symbols={w[3]:{'address':int(w[0],16),'bytes':int(w[1],16)} for line in text.splitlines()
             if len(w:=line.split())==4 and w[2] != 'U'}
    config=make_cfg(symbols,a.prefix)
    a.prefix.parent.mkdir(parents=True,exist_ok=True)
    a.prefix.with_suffix('.cfg').write_text(config,encoding='utf8')
    a.prefix.with_suffix('.symbols.json').write_text(json.dumps(
        {'elf':str(a.elf.resolve()),'elf_sha256':hashlib.sha256(a.elf.read_bytes()).hexdigest(),
         'symbols':{k:symbols[k] for k in EXPECTED},'generated_only':True,
         'connected':False,'reset_or_flash_commands':False},indent=2),encoding='utf8')
    print(a.prefix.with_suffix('.cfg'))
