"""冻结候选并审计真实GPIO分支、ISR/定时采样、输出门禁及试验许可。只读本机ELF。"""
from pathlib import Path
import hashlib,json,re,struct,subprocess,shutil,ast,operator
ROOT=Path(__file__).resolve().parents[1]
HERE=None
BUILD=None
BIN=Path(r'C:/Program Files/Arm/GNU Toolchain mingw-w64-x86_64-arm-none-eabi/bin')
def c_integer(expression):
    expression=re.sub(r'(?<=[0-9a-fA-F])[uUlL]+\b','',expression)
    ops={ast.LShift:operator.lshift,ast.RShift:operator.rshift,ast.BitOr:operator.or_,ast.BitAnd:operator.and_}
    def visit(n):
        if isinstance(n,ast.Constant) and isinstance(n.value,int):return n.value
        if isinstance(n,ast.BinOp) and type(n.op) in ops:return ops[type(n.op)](visit(n.left),visit(n.right))
        raise ValueError('Unexpected macro expression: '+expression)
    return visit(ast.parse(expression,mode='eval').body)
def body(dis,name):
    m=re.search(r'^\w+ <'+re.escape(name)+r'>:\n(.*?)(?=^\w+ <|\Z)',dis,re.M|re.S)
    assert m,name
    return m[1]
def audit(name,compiled=None,hold_ms=0,home_period_ms=2):
    elf=HERE/(name+'.elf');shutil.copy2(BUILD/'robotarm_stm32f407.elf',elf)
    data=elf.read_bytes()
    dis=subprocess.check_output([str(BIN/'arm-none-eabi-objdump.exe'),'-d',str(elf)],text=True)
    (HERE/(name+'_disassembly.txt')).write_text(dis,encoding='utf8')
    nm=subprocess.check_output([str(BIN/'arm-none-eabi-nm.exe'),str(elf)],text=True)
    symbols={p[2]:int(p[0],16) for line in nm.splitlines() if len(p:=line.split())==3 and p[0]!='U'}
    ph=struct.unpack_from('<I',data,28)[0];size,count=struct.unpack_from('<HH',data,42)
    segments=[];vectors=None
    for i in range(count):
        typ,off,_,addr,n,_,_,_=struct.unpack_from('<8I',data,ph+i*size)
        if typ==1 and n and 0x08000000<=addr<0x08080000:
            segments.append({'address':addr,'bytes':n})
            if addr==0x08000000:vectors=data[off:off+400]
    assert vectors
    # 直接从最终ELF读取Service配置，避免协议只改文字、真实周期仍为20ms。
    commands=json.loads((BUILD/'compile_commands.json').read_text())
    parallel=any('-DROBOTARM_PARALLEL_HOMING=1' in x['command'] for x in commands)
    m2_backoff = 1160 if parallel and compiled is None else 1240
    m3_backoff = 160 if parallel and compiled is None else 400
    config_address=symbols.get('s_config')
    if config_address is not None:
        config=None
        for i in range(count):
            typ,off,_,addr,n,_,_,_=struct.unpack_from('<8I',data,ph+i*size)
            if typ==1 and addr<=config_address and config_address+32<=addr+n:
                config=struct.unpack_from('<6IB3xI',data,off+config_address-addr)
                confirm_flag=data[off+config_address-addr+32]
        assert config==(home_period_ms,0,220000,4000,m2_backoff,m3_backoff,4,home_period_ms),config
        # GPIO候选额外开启非当前轴稳定确认，紧随32字节旧配置之后。
        assert confirm_flag==1
    for suffix in ['Core/Src/gpio.c','RobotArm/BSP/src/bsp_limit_irq.c','RobotArm/App/adapters/app_isr_adapter.c','RobotArm/Service/homing/svc_homing_sequence.c']:
        cmd=next(x['command'] for x in commands if x['file'].replace('\\','/').endswith(suffix))
        assert '-DROBOTARM_LIMIT_STATE_MODE=1' in cmd and '-DROBOTARM_SEQUENTIAL_HOMING=1' in cmd
        if parallel:assert '-DROBOTARM_PARALLEL_HOMING=1' in cmd
    if parallel:
        assert b'PARALLEL_HOME' in data and b'PARALLEL:%u STEP_MASK:%u DIR_MASK:%u' in data
        assert b'M412 FRAMES:' in data and b'FIRST_RECORDED:' in data
        if not name.endswith('Release'):assert '<ParallelTick>' in body(dis,'HomeSequence_Tick')
    if not name.endswith('Release'):
        assert '<App_ISR_OnMillisecond>' in body(dis,'SysTick_Handler')
        ms=body(dis,'App_ISR_OnMillisecond')
        assert '<Svc_HomingSequence_OnTickFromISR>' in ms and '<StopForRuntimeLimit>' in ms
        edge=body(dis,'App_ISR_OnGpioEdge')
        assert 'LimitIrqGuard_OnEdgeFromISR' not in edge and 'MotionService_NotifyLimitSwitch' not in edge
        assert '<AppRuntime_RequestModeSwitch>' in edge # 仅限位改普通输入；模式按键仍是EXTI。
        timer=body(dis,'App_ISR_OnTimerElapsed')
        assert timer.index('<StopForRuntimeLimit>')<timer.index('<MotionService_OnStepTickFromISR>')
        gate=body(dis,'StopForRuntimeLimit')
        for target in ['LimitStateFilter_Sample','AppHardware_StopJointSteps','AppHardware_StopActuators','SafetyService_ReportLimitSwitch']:
            assert '<'+target+'>' in gate
        rearm=body(dis,'BSP_RobotBoard_TryRearmLimitInterrupt')
        assert rearm.count('<BSP_RobotBoard_IsLimitTriggered>')>=2
        for fn in ['BSP_RobotBoard_TryRearmLimitInterrupt','BSP_RobotBoard_ResetLimitInterrupts']:
            if fn in symbols: # 未被该入口调用的复位端口可能已被链接器剔除。
                assert not re.search(r'\borr(?:s|\.w)?\s',body(dis,fn)),fn+' must not enable any EXTI bit'
    if compiled is not None:
        assert ('PROTOCOL:STARTUP'+('9' if hold_ms else '8')).encode() in data and b'LIMIT_STATE_1MS' in data
        assert b'STEP_US:%lu BACKOFF_STEP_US:%lu' in data or (
            home_period_ms==2 and b'STEP_US:2000 BACKOFF_STEP_US:2000' in data)
        order='PARALLEL_BARRIER' if parallel else '0,1,2'
        assert ('DEADLINE_MS:220000 BACKOFF:4000,1240,400 SEQUENCE:'+order+' DIR:0,0,1').encode() in data
        assert '<__wrap_HAL_GPIO_WritePin>' in body(dis,'Drv_Stepper_WriteStepLevel')
        assert '<__wrap_HAL_IncTick>' in body(dis,'SysTick_Handler')
        assert 'Dev_Joint_Step' not in symbols
        assert struct.unpack_from('<I',vectors,(16+50)*4)[0]&~1==symbols['TIM5_IRQHandler']&~1
        for i,fn in [(2,'NMI_Handler'),(3,'HardFault_Handler'),(4,'MemManage_Handler'),(5,'BusFault_Handler'),(6,'UsageFault_Handler')]:
            assert struct.unpack_from('<I',vectors,i*4)[0]&~1==symbols[fn]&~1
            assert '<StartupTrial_FailFromException>' in body(dis,fn)
        guard=next(x['command'] for x in commands if x['file'].replace('\\','/').endswith('tools/startup_trial/guard.c'))
        assert f'-DSTARTUP_TRIAL_MOTION_ENABLED={int(compiled)}' in guard
        if not compiled:assert 'HAL_TIM_PWM_Start' not in symbols
        if hold_ms:
            assert f'-DSTARTUP_TRIAL_READY_HOLD_MS={hold_ms}' in guard
            assert f'READY_HOLD_MS:{hold_ms}'.encode() in data and b'READY_HOLD,M128' in data
            assert '<ReadyHold_Arm>' in body(dis,'__wrap_AppSerial_SetMode')
            assert '<ReadyHold_Tick>' in body(dis,'__wrap_main')
            assert '<Stop>' in body(dis,'__wrap_HAL_GPIO_WritePin')
            if compiled:assert '<Stop>' in body(dis,'__wrap_HAL_TIM_PWM_Start')
    record={'build':name,'sha256':hashlib.sha256(data).hexdigest(),'load_bytes':sum(x['bytes'] for x in segments),
            'segments':segments,'compiled_motion':compiled,'state_sampling_hooks_verified':True,
            'flashed':False,'ready_hold_ms':hold_ms,'automatic_startup':compiled is None,'actual_sampling_latency_measured':False,
            'parallel_contact_barrier':parallel,'PS2_diagnostics_query':b'M412 FRAMES:' in data,
            'home_backoff_steps':[4000,m2_backoff,m3_backoff],'home_seek_period_ms':home_period_ms,'home_backoff_period_ms':home_period_ms}
    (HERE/(name+'_audit.json')).write_text(json.dumps(record,indent=2),encoding='utf8')
    return record


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--name',required=True);p.add_argument('--build-dir',required=True);p.add_argument('--output',required=True)
    p.add_argument('--compiled',choices=['0','1']);p.add_argument('--hold-ms',type=int,default=0)
    p.add_argument('--home-period-ms',type=int,choices=[1,2],default=2)
    a=p.parse_args();HERE=Path(a.output).resolve();HERE.mkdir(parents=True,exist_ok=True);BUILD=Path(a.build_dir).resolve()
    r=audit(a.name,None if a.compiled is None else a.compiled=='1',a.hold_ms,a.home_period_ms)
    print(r['build'],r['load_bytes'],'Flash load bytes audited')
