#!/usr/bin/env python3
"""全量软件验收与交付：只编译、运行电脑测试、审计ELF及打包，不接任何设备。

输出新目录，拒绝覆盖已有报告。正常生产/试验/历史构建均编译；只有显式GPIO
Production入口作为推荐候选。包内源文件、固件、清单逐项SHA256核对。
"""
from pathlib import Path
from datetime import datetime,timezone,timedelta
from concurrent.futures import ThreadPoolExecutor
import argparse,ast,hashlib,json,os,shutil,struct,subprocess,sys,threading,zipfile

ROOT=Path(__file__).resolve().parents[1]
ZONE=timezone(timedelta(hours=8))
PRODUCTION=['ProductionLocked','ProductionTrial','ProductionHoldLocked','ProductionHoldTrial','ProductionDebug','ProductionRelease']
OMIT={'.pyc','.o','.obj','.elf','.bin','.hex','.exe','.a','.zip'}

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,x):Path(p).write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def sources(exclude=None):
    paths=set()
    for d in ['RobotArm','Core','Drivers','cmake','tests','scripts','tools','.vscode']:
        for p in (ROOT/d).rglob('*'):
            if p.is_file() and p.suffix not in OMIT and not any(x in p.parts for x in ['__pycache__','.git','build']):paths.add(p)
    # 历史Markdown保留说明，不把历次Flash/ELF/巨量反汇编混入当前代码包。
    paths.update((ROOT/'docs').rglob('*.md'))
    paths.update((ROOT/'docs/engineering_history/20261022_software_final_139').glob('*'))
    paths.update(ROOT/n for n in ['README.md','AGENTS.md','CMakeLists.txt','CMakePresets.json','.gitignore',
        'robotarm_stm32f407.ioc','startup_stm32f407xx.s','STM32F407VETx_FLASH.ld','docs/hardware_validation/latest.json'])
    return sorted(p for p in paths if p.is_file() and (exclude is None or not p.is_relative_to(exclude)))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',help='工程内全新报告目录；默认按北京时间生成')
    a=p.parse_args();stamp=datetime.now(ZONE).strftime('%Y%m%d_%H%M%S')
    out=(ROOT/(a.output or f'docs/software_validation/software_final_{stamp}')).resolve()
    if not out.is_relative_to(ROOT):raise ValueError('report directory must stay inside this project')
    out.mkdir(parents=True,exist_ok=False);(out/'logs').mkdir();(out/'firmware').mkdir()
    lock=threading.Lock();env=os.environ.copy()
    for part in [r'D:\mingw64\bin',r'C:\Program Files\CMake\bin',r'C:\Program Files\Ninja',
                 r'C:\Program Files\Arm\GNU Toolchain mingw-w64-x86_64-arm-none-eabi\bin',
                 r'C:\Users\PC\.cache\codex-runtimes\codex-primary-runtime\dependencies\native\powershell']:
        if Path(part).is_dir():env['PATH']=part+os.pathsep+env.get('PATH','')
    def find(name):
        x=shutil.which(name,path=env['PATH'])
        if not x:raise RuntimeError('Required local tool not found: '+name)
        return x
    cmake=find('cmake');ctest=find('ctest');objcopy=find('arm-none-eabi-objcopy');ninja=find('ninja')
    report={'stage':139,'started_at':datetime.now(ZONE).isoformat(),'commands':[],
        'hardware_operations':{'serial_opened':False,'swd_connected':False,'flashed':False,'reset':False,'motion':False},
        'software_scope_complete':False,'hardware_validation_complete':False,
        'last_verified_hardware_stage':137,'board_software_stage':136,'UART':115200}
    def run(name,cmd,expect=0):
        result=subprocess.run([str(x) for x in cmd],cwd=ROOT,env=env,capture_output=True,text=True,encoding='utf8',errors='replace')
        log=out/'logs'/(name+'.log');log.write_text(result.stdout+result.stderr,encoding='utf8')
        with lock:
            report['commands'].append({'name':name,'argv':[str(x) for x in cmd],'exit_code':result.returncode,'log':log.relative_to(out).as_posix()})
        if (expect==0 and result.returncode) or (expect!=0 and result.returncode==0):
            raise RuntimeError(name+' unexpected exit\n'+(result.stdout+result.stderr)[-3500:])
        return result.stdout+result.stderr
    try:
        before={x.relative_to(ROOT).as_posix():sha(x) for x in sources(out)}
        hardware_sha=sha(ROOT/'docs/hardware_validation/latest.json')
        # 所有手写文件有职责说明；只检查自有源码，不给第三方HAL补教学注释。
        handwritten=[x for x in (ROOT/'RobotArm').rglob('*') if x.suffix in {'.c','.h'}]
        for f in handwritten:
            text=f.read_text(encoding='utf-8-sig')
            if '@file' not in text or '@brief' not in text or '\ufffd' in text:raise RuntimeError('Comment/encoding check failed: '+str(f))
        for d in ['scripts','tools/host','tools/startup_trial','tests']:
            for f in (ROOT/d).rglob('*.py'):ast.parse(f.read_text(encoding='utf-8-sig'),filename=str(f))
        uart=(ROOT/'Core/Src/usart.c').read_text(encoding='utf8')
        import re
        for field,value in {'BaudRate':'115200','WordLength':'UART_WORDLENGTH_8B','StopBits':'UART_STOPBITS_1','Parity':'UART_PARITY_NONE'}.items():
            if not re.search(r'huart1\.Init\.'+field+r'\s*=\s*'+value+r'\s*;',uart):raise RuntimeError('UART1 contract changed: '+field)
        ioc=(ROOT/'robotarm_stm32f407.ioc').read_text(encoding='utf8')
        for pin in ['PA9','PA10']:
            if pin+'.GPIO_Speed=GPIO_SPEED_FREQ_LOW' not in ioc:raise RuntimeError('UART GPIO speed changed')
        run('architecture',[sys.executable,'-B','scripts/check_architecture_dependencies.py'])
        print('Source comments, Python syntax,115200 contract and architecture passed',flush=True)
        builds=ROOT/'build/Software139';builds.mkdir(parents=True,exist_ok=True)
        for preset in ['HostTests','HostTestsRelease']:
            b=builds/preset
            run(preset+'_configure',[cmake,'--preset',preset,'-B',b,f'-DPython3_EXECUTABLE={sys.executable}'])
            run(preset+'_build',[cmake,'--build',b,'-j8'])
            inventory=json.loads(run(preset+'_inventory',[ctest,'--test-dir',b,'--show-only=json-v1']))
            run(preset+'_tests',[ctest,'--test-dir',b,'--output-on-failure','-j8','--output-junit',out/(preset+'_junit.xml')])
            report[preset]={'test_count':len(inventory['tests']),'names':[x['name'] for x in inventory['tests']]}
            print(preset,report[preset]['test_count'],'passed',flush=True)
        for stage in range(8):run('stage'+str(stage),[cmake,'--build',builds/'HostTests','--target','check_stage'+str(stage)])
        print('All8 dependency-stage targets passed',flush=True)
        presets=json.loads((ROOT/'CMakePresets.json').read_text(encoding='utf8'))
        names=[x['name'] for x in presets['configurePresets'] if not x.get('hidden') and x['name'] not in ['HostTests','HostTestsRelease']]
        def firmware(name):
            b=builds/name
            run(name+'_configure',[cmake,'--preset',name,'-B',b,f'-DPython3_EXECUTABLE={sys.executable}'])
            run(name+'_build',[cmake,'--build',b,'-j4'])
            elf=b/'robotarm_stm32f407.elf'
            if not elf.is_file():raise RuntimeError('ELF missing: '+name)
            dest=out/'firmware'/(name+'.elf');shutil.copy2(elf,dest)
            for fmt,ext in [('binary','.bin'),('ihex','.hex')]:run(name+'_'+fmt,[objcopy,'-O',fmt,elf,dest.with_suffix(ext)])
            record={'name':name,'sha256':sha(dest),'elf_bytes':dest.stat().st_size,
                    'recommended_entry':name in PRODUCTION,'hardware_verified':False}
            if name in PRODUCTION:
                args=[sys.executable,'-B','scripts/audit_firmware.py','--name',name,'--build-dir',b,'--output',out/'firmware']
                if name not in ['ProductionDebug','ProductionRelease']:
                    args+=['--compiled','0' if name.endswith('Locked') else '1']
                if 'Hold' in name:args+=['--hold-ms','10000']
                run(name+'_audit',args)
                record['audit']=json.loads((out/'firmware'/(name+'_audit.json')).read_text(encoding='utf8'))
            print(name,'built'+(' and audited' if name in PRODUCTION else ' (reference)'),flush=True)
            return record
        with ThreadPoolExecutor(max_workers=3) as pool:report['firmware']=list(pool.map(firmware,names))
        # 覆盖没有公开预设的两个保留隔离入口。
        for name,options in [('ExtraTmcBench',{'ROBOTARM_TMC_BENCH_MODE':'ON'}),
                             ('ExtraContactTrial',{'ROBOTARM_STARTUP_TRIAL_MODE':'ON','ROBOTARM_LIMIT_SEEK_TRIAL':'ON'})]:
            b=builds/name
            run(name+'_configure',[cmake,'-S',ROOT,'-B',b,'-G','Ninja',f'-DCMAKE_MAKE_PROGRAM={ninja}',
                '-DCMAKE_BUILD_TYPE=Debug',*[f'-D{k}={v}' for k,v in options.items()]])
            run(name+'_build',[cmake,'--build',b,'-j4'])
            target=out/'firmware'/(name+'.elf');shutil.copy2(b/'robotarm_stm32f407.elf',target)
            report['firmware'].append({'name':name,'sha256':sha(target),'recommended_entry':False,'hardware_verified':False})
        invalid=[('bad_hold_value',['-DSTARTUP_TRIAL_READY_HOLD_MS=10001'],'READY hold must be'),
                 ('bad_hold_text',['-DSTARTUP_TRIAL_READY_HOLD_MS=abc'],'READY hold must be'),
                 ('hold_without_trial',['-DSTARTUP_TRIAL_READY_HOLD_MS=10000'],'READY hold requires'),
                 ('gpio_without_sequence',['-DROBOTARM_LIMIT_STATE_MODE=ON'],'Limit state sampling requires')]
        for name,flags,message in invalid:
            text=run(name,[cmake,'-S',ROOT,'-B',builds/name,'-G','Ninja',f'-DCMAKE_MAKE_PROGRAM={ninja}',*flags],expect=1)
            if message not in text:raise RuntimeError('Invalid profile rejected for wrong reason: '+name)
        run('host_cli_offline',[sys.executable,'-B','tools/host/robotarm_client.py'])
        # 生成默认未授权的下一轮材料；不创建部署许可、START claim或运行生成的命令。
        for hold in [0,10000]:run('prepare_acceptance_'+str(hold),[sys.executable,'-B','tools/startup_trial/prepare_acceptance.py',
            '--firmware-dir',out/'firmware','--output',out/('acceptance_hold' if hold else 'acceptance_trial'),'--hold-ms',str(hold)])
        ps=find('pwsh') if shutil.which('pwsh',path=env['PATH']) else find('powershell')
        checker=out/'check_ps_syntax.ps1'
        checker.write_text('''param([string]$Root,[string]$Materials)
$ErrorActionPreference='Stop'
$files=@(Get-ChildItem -LiteralPath (Join-Path $Root 'tools/startup_trial') -Filter '*.ps1')+@(Get-ChildItem -LiteralPath $Materials -Filter '*.ps1' -Recurse)
foreach($f in $files){$e=$null;$t=$null;[void][Management.Automation.Language.Parser]::ParseFile($f.FullName,[ref]$t,[ref]$e);if($e){throw $e}}
"PowerShell syntax passed: $($files.Count) files"
''',encoding='utf8')
        run('powershell_syntax',[ps,'-NoProfile','-File',checker,'-Root',ROOT,'-Materials',out])
        # 默认未授权执行必须在开串口之前拒绝；只测试拒绝分支。
        text=run('unapproved_execution_rejected',[ps,'-NoProfile','-File',out/'acceptance_hold/execute_once.ps1',
            '-EvidenceDirectory',out/'acceptance_hold'],expect=1)
        if 'no port opened' not in text:raise RuntimeError('Unapproved execution did not fail at expected guard')
        after={x.relative_to(ROOT).as_posix():sha(x) for x in sources(out)}
        if before!=after:raise RuntimeError('Sources changed during final validation')
        if sha(ROOT/'docs/hardware_validation/latest.json')!=hardware_sha:raise RuntimeError('Hardware evidence changed')
        report.update(software_scope_complete=True,hardware_validation_complete=False,
            source_file_count=len(before),handwritten_c_h_count=len(handwritten),architecture_passed=True,
            host_uart={'baud':115200,'format':'8N1','gpio_speed':'LOW'},
            debug_tests=report['HostTests']['test_count'],release_tests=report['HostTestsRelease']['test_count'],
            firmware_count=len(report['firmware']),ELF_audit_count=6,invalid_configuration_checks=4,
            backup='docs/software_validation/20261022_software_complete_139/before_edit.zip',
            pending_hardware_plan='docs/hardware_validation/acceptance_plan_139.md',
            completed_at=datetime.now(ZONE).isoformat(),hardware_latest_sha256=hardware_sha)
        save(out/'source_manifest.json',before);save(out/'report.json',report)
        package_dir=ROOT/'deliverables'/('software_139_'+stamp);package_dir.mkdir(parents=True,exist_ok=False)
        package=package_dir/'RobotArm_Software_139.zip';entries={}
        with zipfile.ZipFile(package,'x',zipfile.ZIP_DEFLATED) as z:
            def add(path,name):z.write(path,name);entries[name]=sha(path)
            for relative in before:add(ROOT/relative,'robotarm_stm32f407/'+relative)
            for f in (out/'firmware').iterdir():
                if f.suffix in ['.elf','.bin','.hex','.json']:add(f,'firmware/'+f.name)
            for n in ['report.json','source_manifest.json', 'HostTests_junit.xml','HostTestsRelease_junit.xml']:
                add(out/n,'validation/'+n)
            for f in (out/'logs').glob('*.log'):add(f,'validation/logs/'+f.name)
            for d in ['acceptance_trial','acceptance_hold']:
                for f in (out/d).iterdir():
                    if f.is_file():add(f,'prepared_materials/'+d+'/'+f.name)
            # 导出包内最新软件索引；工作区hardware最新证据不改。
            add(out/'report.json','robotarm_stm32f407/docs/software_validation/latest.json')
            save(out/'package_manifest.json',entries);add(out/'package_manifest.json','validation/package_manifest.json')
        with zipfile.ZipFile(package) as z:
            if z.testzip():raise RuntimeError('ZIP CRC failure')
            for name,digest in entries.items():
                if hashlib.sha256(z.read(name)).hexdigest()!=digest:raise RuntimeError('ZIP hash mismatch: '+name)
        receipt={'stage':139,'report':str((out/'report.json').relative_to(ROOT)),
            'report_sha256':sha(out/'report.json'),'package':str(package.relative_to(ROOT)),
            'package_sha256':sha(package),'entries_verified':len(entries),
            'software_scope_complete':True,'hardware_validation_complete':False,
            'debug_tests':report['debug_tests'],'release_tests':report['release_tests'],
            'firmware_count':report['firmware_count'],'ELF_audit_count':6,'last_verified_hardware_stage':137,
            'board_software_stage':136,'flashed':False,'serial_opened':False,'swd_connected':False,'hardware_actions_performed':False}
        save(out/'receipt.json',receipt);save(package_dir/'receipt.json',receipt)
        (package_dir/'SHA256.txt').write_text(receipt['package_sha256']+'  '+package.name+'\n',encoding='utf8')
        save(ROOT/'docs/software_validation/latest.json',receipt)
        print(json.dumps(receipt,ensure_ascii=False),flush=True);return 0
    except Exception as exc:
        report['software_scope_complete']=False;report['error']=str(exc);save(out/'report.json',report);raise

if __name__=='__main__':raise SystemExit(main())
