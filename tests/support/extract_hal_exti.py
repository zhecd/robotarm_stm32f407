"""提取仓库原厂HAL中断函数原文供主机编译；不复制改写其实现，不连接设备。"""
import hashlib
import json
from pathlib import Path
import re
import sys

source, header, output = map(Path, sys.argv[1:])
text = source.read_text(encoding='utf-8')
match = re.search(r'^void HAL_GPIO_EXTI_IRQHandler\(uint16_t GPIO_Pin\)\s*\{', text, re.M)
if match is None:
    raise ValueError('HAL handler signature changed; review extraction')
depth = 1
end = match.end()
while depth and end < len(text):
    depth += (text[end] == '{') - (text[end] == '}')
    end += 1
if depth:
    raise ValueError('Incomplete HAL handler')
body = text[match.start():end]
clear = re.search(r'^#define __HAL_GPIO_EXTI_CLEAR_IT\(__EXTI_LINE__\)\s+(.+)$',
                  header.read_text(encoding='utf-8'), re.M)
if clear is None or clear[1].strip() != '(EXTI->PR = (__EXTI_LINE__))':
    raise ValueError('HAL W1C macro changed; review test register model')
output.write_text('/* Build-generated original HAL function; register macros use explicit test ports. */\n'
                  '#include "main.h"\n' + body + '\n', encoding='utf-8')
output.with_suffix('.json').write_text(json.dumps({
    'source': str(source), 'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'function_sha256': hashlib.sha256(body.encode('utf-8')).hexdigest(),
    'original_function_unchanged': True, 'verified_W1C_macro': clear[1].strip(),
    'whole_HAL_or_hardware_emulated': False}, indent=2), encoding='utf-8')
