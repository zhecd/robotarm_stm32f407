"""源码依赖检查；覆盖直接 include、服务私有域和公共头文件的间接 HAL 泄漏。"""
from __future__ import annotations
import pathlib
import re

LEXEMES=re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',re.S)
INCLUDE=re.compile(r'^\s*#\s*include\s*"([^"]+)"',re.M)
HARDWARE=re.compile(r'\b(?:HAL_\w*|GPIO_\w*|\w+_HandleTypeDef|__disable_irq|__get_PRIMASK)\b')
CORE_HEADERS={'main.h','usart.h','tim.h','i2c.h','gpio.h','dma.h','stm32f4xx_hal.h'}
ALLOWED={
    'Algorithm':{'Algorithm'}, 'Platform':{'Platform'},
    'BSP':{'BSP','Algorithm','Platform'},
    'Device':{'Device','BSP','Algorithm','Platform'},
    'Service':{'Service','Device','Algorithm','Platform'},
    'App':{'App','Service','Algorithm','Platform'},
}

def strip_comments(source: str) -> str:
    """保留字符串中的 //；注释以换行替换，便于后续定位。"""
    return LEXEMES.sub(lambda m:'\n'*m[0].count('\n') if m[0].startswith('/') else m[0],source)

def code_tokens(source: str) -> str:
    return LEXEMES.sub(lambda m:'' if m[0].startswith(('"',"'")) else m[0],source)

def check_tree(root: pathlib.Path) -> list[str]:
    root=root.resolve(); robot=root/'RobotArm'
    if not robot.is_dir(): return ['RobotArm directory is missing']
    files=sorted(p for p in robot.rglob('*') if p.suffix in {'.c','.h'})
    headers=[p for p in files if p.suffix=='.h']
    core=root/'Core'/'Inc'
    if core.is_dir(): headers+=list(core.rglob('*.h'))
    by_name:dict[str,list[pathlib.Path]]={}
    for path in headers: by_name.setdefault(path.name,[]).append(path)
    cleaned={p:strip_comments(p.read_text(encoding='utf-8-sig')) for p in files+headers}
    errors:set[str]=set()
    def layer(path):
        try: return path.relative_to(robot).parts[0]
        except ValueError: return None
    def resolve(path,inc):
        local=(path.parent/inc).resolve()
        if local in cleaned: return local
        suffix=inc.replace('\\','/').lstrip('./')
        candidates=by_name.get(pathlib.PurePosixPath(suffix).name,[])
        exact=[p for p in candidates if p.as_posix().endswith('/'+suffix)]
        return exact[0] if len(exact)==1 else candidates[0] if len(candidates)==1 else None
    def add(path,message): errors.add(f'{path.relative_to(root).as_posix()}: {message}')
    for path in files:
        owner=layer(path); text=cleaned[path]
        adapter=owner=='App' and 'adapters' in path.relative_to(robot).parts
        if owner in {'Algorithm','Device','Service','App'} and not adapter and HARDWARE.search(code_tokens(text)):
            add(path,'hardware types/calls must stay in BSP or the explicit ISR adapter')
        for inc in INCLUDE.findall(text):
            target=resolve(path,inc); target_layer=layer(target) if target else None
            if target_layer and target_layer not in ALLOWED.get(owner,set()) and not adapter:
                add(path,f'forbidden layer dependency {owner} -> {target_layer}: {inc}')
            if owner in {'Algorithm','Device','Service','App'} and not adapter and pathlib.PurePosixPath(inc).name in CORE_HEADERS:
                add(path,f'forbidden CubeMX include {inc}')
            if target and 'internal' in target.parts:
                domain=target.relative_to(robot).parts[1]
                same=owner=='Service' and path.relative_to(robot).parts[1]==domain
                if not same: add(path,f'private implementation include {inc}')
            if owner=='Service' and path.name=='safety_service.c' and inc=='motion_service.h':
                add(path,'Safety must not depend on Motion')
        public=path.suffix=='.h' and 'include' in path.relative_to(robot).parts and owner in {'App','Service','Device','Algorithm'}
        if public:
            stack=[path]; seen=set()
            while stack:
                current=stack.pop()
                if current in seen: continue
                seen.add(current); content=cleaned[current]
                if HARDWARE.search(code_tokens(content)):
                    add(path,f'public header leaks hardware through {current.relative_to(root).as_posix()}')
                for inc in INCLUDE.findall(content):
                    if pathlib.PurePosixPath(inc).name in CORE_HEADERS or 'stm32' in inc.lower():
                        add(path,f'public header indirectly includes hardware: {inc}')
                    target=resolve(current,inc)
                    if target: stack.append(target)
    return sorted(errors)
