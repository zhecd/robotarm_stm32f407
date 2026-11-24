"""在已授权且执行器断开的现场执行最小对照，随后恢复已验证波形固件；不发送动作。"""
from pathlib import Path
from datetime import datetime,timezone,timedelta
import argparse,hashlib,json,re,socket,struct,subprocess,time

ROOT=Path(__file__).resolve().parents[2]
EXPECTED_WAVE='69214425d3cb344fa9b2546caabec314aa6c990c4e2321d268578457cada09e1'
STATE_WORDS=115
STATS_WORDS=26
RECORD_WORDS=13

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def segments(path):
    b=path.read_bytes();assert b[:6]==b'\x7fELF\x01\x01'
    offset=struct.unpack_from('<I',b,28)[0];size,count=struct.unpack_from('<HH',b,42)
    result=[]
    for i in range(count):
        kind,at,virtual,physical,n,memory,flags,align=struct.unpack_from('<8I',b,offset+i*size)
        if kind==1 and n:
            assert 0x08000000<=physical<physical+n<=0x08080000
            result.append((physical,b[at:at+n]))
    assert result;return result
def decode_state(blob):
    words=struct.unpack('<115I',blob)
    names=('magic','version','ready','done','clock_hz','ms','phase','phase_start','trace_count','trace_dropped','clock_error')
    data=dict(zip(names,words[:11]));data['stats']=[]
    for p in range(4):
        w=words[11+p*STATS_WORDS:11+(p+1)*STATS_WORDS]
        data['stats'].append({'phase':p,'samples':w[0],'low_samples':w[1:4],
            'falls':w[4:7],'rises':w[7:10],'pending':w[10:13],'after_ack':w[13:16],
            'irq_vectors':w[16],'spurious_vectors':w[17],'masked':w[18],
            'guard_count':w[19:22],'guard_window':w[22:25],'last_low_mask':w[25]})
    return data

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-authorized-isolated-test',action='store_true',required=True,
                        help='Only with existing authorization and motors/gripper physically disconnected')
    parser.add_argument('--probe-preset',choices=('TeachingExtiProbe','TeachingExtiProbeUart'),default='TeachingExtiProbe')
    parser.add_argument('--current-waveform-baseline',action='store_true',help='Verify and restore the latest validated EN-locked waveform image')
    parser.add_argument('--inject-exti-control',action='store_true',help='One labelled SWIER control near end of IRQ phase; exclude control tail from natural-event conclusions')
    args=parser.parse_args();assert args.run_authorized_isolated_test
    latest=json.loads((ROOT/'docs/validation/latest.json').read_text(encoding='utf-8'))
    report_path=ROOT/latest['report'];assert sha(report_path)==latest['report_sha256']
    report=json.loads(report_path.read_text(encoding='utf-8'));assert report['software_validation_passed']
    fw=next(x for x in report['firmware'] if x['preset']==args.probe_preset)
    is_uart=args.probe_preset=='TeachingExtiProbeUart'
    probe=ROOT/fw['path'];assert sha(probe)==fw['sha256']
    manifest=json.loads((report_path.parent/'source_manifest.json').read_text(encoding='utf-8'))
    for file in ('tools/limit_probe/main.c','tools/limit_probe/probe_logic.c','tools/limit_probe/probe_logic.h','tools/limit_probe/run_board.py','tools/limit_probe/probe_console.c','tools/limit_probe/probe_console.h','tools/limit_probe/query_uart.ps1'):
        assert sha(ROOT/file)==manifest[file],file+' changed after validation'
    wave=ROOT/'docs/hardware_validation/20261019_wave5_vector_trace_01/TeachingWaveform.elf'
    expected_wave=EXPECTED_WAVE
    if args.current_waveform_baseline:
        baseline=next(x for x in report['firmware'] if x['preset']=='TeachingWaveform')
        wave=ROOT/baseline['path'];expected_wave=baseline['sha256']
    assert sha(wave)==expected_wave
    suffix='_minimal_exti_probe_uart' if is_uart else '_minimal_exti_probe'
    here=ROOT/'docs/hardware_validation'/(datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')+suffix)
    here.mkdir(exist_ok=False)
    probe_frozen=here/'TeachingExtiProbe.elf';probe_frozen.write_bytes(probe.read_bytes())
    wave_frozen=here/'BaselineWaveform.elf';wave_frozen.write_bytes(wave.read_bytes())
    nm=r'C:\Program Files\Arm\GNU Toolchain mingw-w64-x86_64-arm-none-eabi\bin\arm-none-eabi-nm.exe'
    symbols=subprocess.run([nm,'-n','-S',str(probe_frozen)],capture_output=True,text=True,check=True).stdout
    baseline_symbols=subprocess.run([nm,'-n','-S',str(wave_frozen)],capture_output=True,text=True,check=True).stdout
    assert 'AppWaveform_Init' in baseline_symbols and 'Svc_Homing_Start' not in baseline_symbols and 'Dev_Gripper_Init' not in baseline_symbols
    (here/'symbols.txt').write_text(symbols,encoding='utf-8')
    for forbidden in ('HAL_GPIO_EXTI_IRQHandler','Svc_Homing_Start','Dev_Gripper_Init','App_RunOnce'):
        assert not re.search(r'\b'+forbidden+r'$',symbols,re.M),forbidden
    def address(name,size):
        m=re.search(rf'(?m)^([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+\w\s+{name}$',symbols)
        assert m and int(m[2],16)==size,(name,m)
        return int(m[1],16)
    state_address=address('g_limit_probe',STATE_WORDS*4)
    trace_address=address('g_limit_probe_trace',128*RECORD_WORDS*4)
    record={'started_at_utc':datetime.now(timezone.utc).isoformat(),'source_report':str(report_path.relative_to(ROOT)),
        'source_report_sha256':latest['report_sha256'],'probe_sha256':sha(probe_frozen),
        'restore_sha256':expected_wave,'probe_preset':args.probe_preset,'motors_and_gripper':'physically disconnected per standing user setup',
        'external_limits':'M3 reconnected per latest user message; M1/M2 unchanged',
        'serial_commands_sent':is_uart,'normal_firmware_started':False,'mechanical_motion_requested':False,
        'operations':[],'snapshots':[],'result':'prepared'}
    path=here/'stage_record.json';save(path,record)
    try:
        with socket.create_connection(('127.0.0.1',6666),timeout=1):
            raise RuntimeError('Existing debug server; do not reuse unknown session')
    except (ConnectionRefusedError,TimeoutError):pass
    cfg=here/'attach.cfg'
    cfg.write_text('''source [find interface/stlink.cfg]
transport select dapdirect_swd
source [find target/stm32f4x.cfg]
reset_config none
cortex_m reset_config sysresetreq
adapter speed 1000
stm32f4x.cpu configure -event examine-end {}
stm32f4x.cpu configure -event reset-start { adapter speed 1000 }
bindto 127.0.0.1
gdb_port disabled
telnet_port disabled
tcl_port 6666
init
echo "LIMIT_PROBE_ATTACH_READY"
''',encoding='utf-8')
    with (here/'openocd.log').open('xb') as log:
        proc=subprocess.Popen([r'C:\Program Files\xpack-openocd-0.12.0-7\bin\openocd.exe',
            '-s',r'C:\Program Files\xpack-openocd-0.12.0-7\openocd\scripts','-f',str(cfg)],
            stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
    save(here/'openocd_pid.json',{'pid':proc.pid})
    def rpc(command):
        operation={'time_utc':datetime.now(timezone.utc).isoformat(),'command':command}
        record['operations'].append(operation);save(path,record)
        with socket.create_connection(('127.0.0.1',6666),timeout=10) as s:
            s.settimeout(30);s.sendall((command+'\x1a').encode());data=bytearray()
            while b'\x1a' not in data:
                block=s.recv(65536)
                if not block:
                    if command=='shutdown':break
                    raise RuntimeError('Incomplete command reply; do not automatically resend')
                data.extend(block)
        reply=data.split(b'\x1a',1)[0].decode(errors='replace').strip()
        operation['reply']=reply;save(path,record);return reply
    def read_words(addr,n):
        result=rpc(f'capture {{mdw 0x{addr:08x} {n}}}')
        words=[]
        for line in result.splitlines():
            m=re.fullmatch(r'0x[0-9a-fA-F]+:\s*((?:[0-9a-fA-F]{8}\s*)+)',line)
            assert m,result
            words.extend(int(x,16) for x in m[1].split())
        assert len(words)==n,(result,n);return words
    def verify_flash(image,label):
        parts=segments(image);start=min(a for a,b in parts);end=max(a+len(b) for a,b in parts)
        out=here/(label+'_flash_readback.bin');assert not out.exists()
        rpc(f'dump_image {{{out.as_posix()}}} 0x{start:08x} {end-start}')
        actual=out.read_bytes();assert len(actual)==end-start
        checks=[{'address':hex(a),'bytes':len(b),'match':actual[a-start:a-start+len(b)]==b} for a,b in parts]
        record[label+'_flash']={'segments':checks,'readback_sha256':sha(out)};save(path,record)
        assert all(c['match'] for c in checks),label+' Flash mismatch'
    def program(image,label):
        rpc('reset halt');assert rpc('stm32f4x.cpu curstate')=='halted'
        record['phase']=label;save(path,record)
        rpc(f'flash write_image erase {{{image.as_posix()}}}')
        rpc(f'verify_image {{{image.as_posix()}}}')
        verify_flash(image,label)
        rpc('reset halt');assert rpc('stm32f4x.cpu curstate')=='halted'
        rpc('resume');assert rpc('stm32f4x.cpu curstate')=='running'
    uart_proc=None
    try:
        for _ in range(75):
            logtext=(here/'openocd.log').read_text(encoding='utf-8',errors='replace')
            if 'LIMIT_PROBE_ATTACH_READY' in logtext:break
            if proc.poll() is not None:raise RuntimeError(logtext)
            time.sleep(0.2)
        else:raise RuntimeError('No attach marker; no automatic reset/retry')
        assert rpc('stm32f4x.cpu curstate')=='running'
        verify_flash(wave_frozen,'before')
        program(probe_frozen,'probe')
        if is_uart:
            with (here/'uart_query.log').open('xb') as uart_log:
                uart_proc=subprocess.Popen(['powershell.exe','-NoProfile','-ExecutionPolicy','Bypass','-File',
                    str(ROOT/'tools/limit_probe/query_uart.ps1'),'-OutputDir',str(here)],
                    stdout=uart_log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        print('AUDIT '+str(here),flush=True)
        started=time.monotonic();last_phase=None;last_report=0
        while True:
            h=read_words(state_address,11)
            assert h[0]==0x4c505231 and h[1]==1 and h[10]==0,('Probe header/clock failed',h)
            record['snapshots'].append({'host_elapsed_s':round(time.monotonic()-started,3),'header':h});save(path,record)
            if args.inject_exti_control and h[6]==2 and h[5]>=57000 and not record.get('software_control_injected'):
                data=read_words(state_address,STATE_WORDS)
                record['natural_prefix_before_control']=decode_state(struct.pack('<115I',*data))
                record['control_time_ms']=h[5];save(path,record)
                rpc('mww 0x40013c10 0x00001000')
                record['software_control_injected']=True;save(path,record)
                print('CONTROL: one EXTI12 SWIER injection; later tail is labelled controlled, not natural',flush=True)
            if uart_proc is not None and uart_proc.poll() is not None:
                assert uart_proc.returncode==0,'UART query profile failed; inspect uart_query.log'
            if h[6]!=last_phase or time.monotonic()-last_report>=10:
                print(f'PROBE phase={h[6]} ms={h[5]} ready={h[2]} done={h[3]} trace={h[8]} dropped={h[9]}',flush=True)
                last_phase=h[6];last_report=time.monotonic()
            if h[3]:break
            assert time.monotonic()-started<100,'Probe phase timeout'
            if time.monotonic()-started>3:assert h[2]==1,'Probe never became ready'
            time.sleep(1)
        assert h[6]==4 and h[4]==168000000
        out=here/'probe_state.bin';rpc(f'dump_image {{{out.as_posix()}}} 0x{state_address:08x} {STATE_WORDS*4}')
        state=decode_state(out.read_bytes());assert state['done']==1 and state['phase']==4
        count=state['trace_count'];assert count<=128
        traces=[]
        names=('ms','phase','source','pending_before','handled','pending_after','gpio_before','gpio_after','imr','falling','rising','exticr4','cycles')
        if count:
            out=here/'probe_trace.bin';rpc(f'dump_image {{{out.as_posix()}}} 0x{trace_address:08x} {count*RECORD_WORDS*4}')
            raw=out.read_bytes();assert len(raw)==count*RECORD_WORDS*4
            traces=[dict(zip(names,struct.unpack_from('<13I',raw,i*52))) for i in range(count)]
        registers={name:read_words(addr,1)[0] for name,addr in (
            ('RCC_CFGR',0x40023808),('GPIOC_IDR',0x40020810),('GPIOC_MODER',0x40020800),
            ('GPIOC_PUPDR',0x4002080c),('EXTICR3',0x40013810),('EXTICR4',0x40013814),
            ('EXTI_PR',0x40013c14),('EXTI_IMR',0x40013c00),('EXTI_FTSR',0x40013c0c),
            ('GPIOD_ODR',0x40020c14),('GPIOA_ODR',0x40020014),('GPIOB_ODR',0x40020414),
            ('TIM2_CCER',0x40000020),('TIM6_CR1',0x40001000))}
        assert registers['GPIOD_ODR']&4 and not registers['GPIOA_ODR']&0xc0 and not registers['GPIOB_ODR']&1
        assert not registers['TIM2_CCER']&16 and not registers['TIM6_CR1']&1
        save(here/'probe_result.json',{'state':state,'traces':traces,'registers':registers})
        if uart_proc is not None:
            uart_proc.wait(timeout=5);assert uart_proc.returncode==0
            uart_record=json.loads((here/'uart_capture.json').read_text(encoding='utf-8-sig'))
            assert uart_record['result']=='query_completed' and uart_record['serial_closed']
        if args.inject_exti_control:
            assert record.get('software_control_injected'),'Control window was not reached'
            before=record['natural_prefix_before_control']['stats'][2]['pending'][2]
            assert state['stats'][2]['pending'][2]>before,'EXTI12 control did not reach IRQ counter'
        record['probe_complete']=True;save(path,record)
        print('PROBE_STATS '+json.dumps(state['stats']),flush=True)
        program(wave_frozen,'restore')
        record['result']='probe_completed_and_baseline_waveform_restored';save(path,record)
    except Exception as error:
        record['result']='failed';record['error']=str(error);save(path,record)
        raise
    finally:
        if proc.poll() is None:
            try:
                record['cpu_before_shutdown']=rpc('stm32f4x.cpu curstate')
                rpc('shutdown');proc.wait(timeout=5);record['debugger_closed']=True
            except Exception as error:record['cleanup_error']=str(error)
        record['completed_at_utc']=datetime.now(timezone.utc).isoformat();save(path,record)
        print('RESULT '+json.dumps({k:v for k,v in record.items() if k not in ('operations','snapshots')},ensure_ascii=False),flush=True)
    return 0
if __name__=='__main__':raise SystemExit(main())
