"""生成下一轮实机验收材料；只写本地文件，绝不打开串口或调用OpenOCD。

默认authorization.approved=false，生成材料不构成动作许可。重复目录拒绝覆盖。
"""
from pathlib import Path
import argparse,hashlib,json,re,shutil,struct

ROOT=Path(__file__).resolve().parents[2]
FEATURES='IRQ_EVIDENCE,SEQUENTIAL_HOME,M126,LIMIT_STATE_1MS,HOME_INPUT_CONFIRM,M127,CONTACT_CANCEL_10MS'

def identity(compiled,hold_ms):
    if type(compiled) is not int or compiled not in (0,1) or hold_ms not in (0,10000):
        raise ValueError('invalid acceptance profile')
    return (f'M115 FIRMWARE:RobotArmStartupTrial PROTOCOL:STARTUP{9 if hold_ms else 8} UART:115200 '
            f'MOTION_COMPILED:{compiled} AUTO_MOTION:0 SEEK_TIMEOUT_MS:0 STEP_US:1000 '
            'BACKOFF_STEP_US:1000 DEADLINE_MS:220000 BACKOFF:4000,1240,400 SEQUENCE:0,1,2 DIR:0,0,1'
            +(f' READY_HOLD_MS:{hold_ms}' if hold_ms else '')+' FEATURES:'+FEATURES
            +(',READY_HOLD,M128' if hold_ms else ''))

def flash_segments(data):
    if len(data)<52 or data[:6]!=b'\x7fELF\x01\x01':raise ValueError('expected 32-bit little-endian ELF')
    ph=struct.unpack_from('<I',data,28)[0];size,n=struct.unpack_from('<HH',data,42)
    if size!=32 or ph+n*size>len(data):raise ValueError('invalid ELF program headers')
    segments=[]
    for i in range(n):
        typ,off,_,addr,length,_,_,_=struct.unpack_from('<8I',data,ph+i*size)
        if typ==1 and length and 0x08000000<=addr<0x08080000:
            if off+length>len(data) or addr+length>0x08080000:raise ValueError('ELF flash range invalid')
            segments.append((addr,off,length))
    if not segments:raise ValueError('no flash load segments')
    return segments

def prepare(output,firmware_dir,port='COM4',hold_ms=0):
    if not re.fullmatch(r'COM[1-9][0-9]*',port):raise ValueError('explicit COM number required')
    if hold_ms not in (0,10000):raise ValueError('hold profile must be0 or10000')
    output=Path(output).resolve();firmware_dir=Path(firmware_dir).resolve()
    if any(c in str(output)+str(firmware_dir) for c in '{}\r\n'):raise ValueError('unsupported Tcl path characters')
    names=['ProductionHoldLocked','ProductionHoldTrial'] if hold_ms else ['ProductionLocked','ProductionTrial']
    candidates={}
    for name in names:
        p=firmware_dir/(name+'.elf');data=p.read_bytes();flash_segments(data)
        audit=json.loads((firmware_dir/(name+'_audit.json')).read_text(encoding='utf8'))
        expected_motion=name.endswith('Trial')
        if audit.get('sha256')!=hashlib.sha256(data).hexdigest() or audit.get('compiled_motion') is not expected_motion:
            raise ValueError('frozen ELF audit mismatch: '+name)
        # 与最终审计的身份保持一致，不接收普通自动启动固件代替单次试验。
        for token in [f'PROTOCOL:STARTUP{9 if hold_ms else 8}'.encode(),b'AUTO_MOTION:0',b'BACKOFF:4000,1240,400']:
            if token not in data:raise ValueError('firmware identity mismatch: '+name)
        candidates[name]={'sha256':hashlib.sha256(data).hexdigest(),'load_segments':flash_segments(data)}
    output.mkdir(parents=True,exist_ok=False)
    def save(n,s): (output/n).write_text(s,encoding='utf8')
    save('authorization.json',json.dumps({'approved':False,'single_START_only':True,
        'status':'prepared_only_not_authorized','user_reply':None,'UART':115200,'port':port,
        'seek_rate':1000,'backoff_rate':1000,'seek_timeout_ms':0,'total_timeout_ms':220000,
        'backoff_steps':[4000,1240,400],'ready_hold_ms':hold_ms,'automatic_retry':False},indent=2))
    common=(ROOT/'tools/startup_trial/serial_common.ps1').read_text(encoding='utf-8-sig')
    common=common.replace("'COM4'",repr(port))
    start=common.index(' if($id -cne "');end=common.index('\n',start)
    expected=identity(1,hold_ms).replace('MOTION_COMPILED:1','MOTION_COMPILED:$compiled')
    common=common[:start]+f' if($id -cne "{expected}"){{throw "Wrong identity: $id"}}'+common[end:]
    save('serial_common.ps1',common)
    shutil.copy2(ROOT/'tools/startup_trial/execute_sequential_after_approval.ps1',output/'execute_once.ps1')
    shutil.copy2(ROOT/'tools/startup_trial/query_status.ps1',output/'query_status.ps1')
    save('preflight_locked.ps1','''$ErrorActionPreference='Stop'
. "$PSScriptRoot/serial_common.ps1"
try {Open-Port;Check-Wait 0;$null=Q 'M119';'Locked preflight passed'} finally {if($p){$p.Close();$p.Dispose()}}
''')
    # 部署命令本身也要求本轮许可及新claim；不能只改脚本就默认批准。
    base='''source [find interface/stlink.cfg]
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
'''
    outputs_disabled='''set en [lindex [read_memory 0x40020c10 32 1] 0]
set a [lindex [read_memory 0x40020010 32 1] 0]
set b [lindex [read_memory 0x40020410 32 1] 0]
set pwm [lindex [read_memory 0x40000000 32 1] 0]
set ccer [lindex [read_memory 0x40000020 32 1] 0]
if {($en&4)==0 || ($a&0xc2)!=0 || ($b&1)!=0 || ($pwm&1)!=0 || ($ccer&0x1111)!=0} {error "OUTPUTS_NOT_DISABLED"}
echo "OUTPUTS_DISABLED"
'''
    save('capture_before.cfg',base+outputs_disabled+f'dump_image {{{(output/"before_flash.bin").as_posix()}}} 0x08000000 0x80000\nshutdown\n')
    for kind,name in zip(['locked','motion'],names):
        elf=firmware_dir/(name+'.elf')
        # 本地审批转换为单独deploy许可文件，不能把“已生成目录”当作许可。
        pre=f'''if {{![file exists {{{(output/'deployment_authorized.claim').as_posix()}}}]}} {{error "DEPLOY_NOT_AUTHORIZED"}}
if {{![file exists {{{(output/'before_flash.bin').as_posix()}}}] || [file size {{{(output/'before_flash.bin').as_posix()}}}]!=524288}} {{error "BACKUP_REQUIRED"}}
set claim [open {{{(output/f'flash_{kind}_once.claim').as_posix()}}} {{WRONLY CREAT EXCL}}]
close $claim
'''
        save(f'flash_{kind}.cfg',pre+base+f'''set old_debug [lindex [read_memory 0xe0042008 32 1] 0]
mww 0xe0042008 [expr {{$old_debug|0x1000}}]
reset halt
flash write_image erase {{{elf.as_posix()}}}
verify_image {{{elf.as_posix()}}}
dump_image {{{(output/f'flash_{kind}.bin').as_posix()}}} 0x08000000 0x80000
mww 0xe0042008 $old_debug
reset run
sleep 500
'''+outputs_disabled+'''shutdown
''')
    save('firmware_manifest.json',json.dumps(candidates,indent=2))
    save('README.md','''# 待授权实机材料

本目录由离线工具生成，approved=false，未连接任何设备。获得本轮明确许可后才能记录授权、创建deployment_authorized.claim、连接并烧录。保留所有claim，不删除重放。

顺序：备份before_flash.bin→部署locked→独立回读比对ELF各Flash段→锁定查询→部署motion→独立回读比对→execute_once.ps1。异常停止，不自动重试。flash_segments()可用于独立比对；不得只相信烧录器显示成功。

动作参数以authorization.json为准。hold配置为10000时，回零后保持使能10秒，继续租约和故障保护，禁止新STEP/PWM，完成断能并撤销参考。该模式不验证普通闭环运动和实际夹物到位。正式自动启动固件另行验收，不能替代本单次镜像。
''')
    return candidates

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True);p.add_argument('--firmware-dir',required=True)
    p.add_argument('--port',default='COM4');p.add_argument('--hold-ms',type=int,choices=[0,10000],default=0)
    a=p.parse_args();prepare(a.output,a.firmware_dir,a.port,a.hold_ms)
    print('Prepared only; approval=false; no device accessed: '+a.output)
if __name__=='__main__':main()
