"""读取已标定坐标格式的示例输入，生成计划并调用电脑端C模型；没有设备连接入口。"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

from vision_sorting_plan import Detection, build_sorting_plan


def finite_number(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def preview_plan(plan, start_mm, checker, timeout_s=30.0):
    """调用显式提供的本机robotarm_plan_check程序，不使用shell或串口。

    起点由用户输入，不暗取历史编码器/零位。本地模型的合成许可不传递给机械臂。
    返回失败模型也保留失败项和部分计数，禁止把部分预演当作整条计划成功。
    """
    if (not isinstance(start_mm, (list, tuple)) or len(start_mm) != 3
            or any(not finite_number(v) for v in start_mm)):
        raise ValueError('start_mm必须为三个有限mm数值')
    if not isinstance(plan, (list, tuple)) or not 1 <= len(plan) <= 256:
        raise ValueError('计划须为1..256个条目')
    if not finite_number(timeout_s) or timeout_s <= 0:
        raise ValueError('预演进程超时须为有限正秒数')
    lines = []
    for item in plan:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            raise ValueError('计划条目必须为种类/值二元组')
        kind, value = item
        if kind == 'command':
            if (not isinstance(value, str) or not 1 <= len(value) <= 255
                    or any(ord(c) < 32 or ord(c) > 126 for c in value)):
                raise ValueError('预演命令须为单行ASCII，最多255字符')
        elif kind == 'wait_seconds':
            if not finite_number(value) or not 0.6 <= value <= 60:
                raise ValueError('夹爪等待须为0.6..60秒')
        else:
            raise ValueError('未知计划条目种类')
        lines.append(f'{kind}\t{value}\n')
    payload = ''.join(lines)
    executable = Path(checker).resolve(strict=True)
    completed = subprocess.run([str(executable), *(str(v) for v in start_mm)],
        input=payload, text=True, encoding='ascii', capture_output=True,
        timeout=timeout_s, shell=False)
    try:
        model = json.loads(completed.stdout)
    except (ValueError, TypeError) as exc:
        raise RuntimeError('C预演器未返回完整JSON结果: ' + completed.stderr[:200]) from exc
    if (completed.returncode not in (0, 2) or not isinstance(model, dict)
            or type(model.get('pass')) is not bool
            or model.get('simulation_only') is not True
            or model.get('hardware_connected') is not False
            or model.get('synthetic_reference') is not True
            or model['pass'] != (completed.returncode == 0)
            or (model['pass'] and model.get('completed_items') != len(plan))):
        raise RuntimeError('C预演器结果合同不符')
    return {'schema': 'robotarm-sorting-preview-v1', 'start_mm': list(start_mm),
            'plan': list(plan), 'model': model,
            'checker_sha256': hashlib.sha256(executable.read_bytes()).hexdigest(),
            'plan_sha256': hashlib.sha256(payload.encode('ascii')).hexdigest()}


def preview_request(data, checker):
    """任务数据→离线计划→真实C业务链模型；不导入RobotArmClient，不查询或控制设备。"""
    expected = {'schema', 'start_mm', 'detection', 'bins', 'safe_z_mm',
                'feed_mm_min', 'gripper_settle_s'}
    if (not isinstance(data, dict) or set(data) != expected
            or data['schema'] != 'robotarm-sorting-preview-v1'
            or not isinstance(data['detection'], dict)
            or set(data['detection']) != {'color', 'x_mm', 'y_mm', 'z_mm'}
            or not isinstance(data['bins'], dict)):
        raise ValueError('分拣输入格式或必需字段不符')
    plan = build_sorting_plan(Detection(**data['detection']), data['bins'], data['safe_z_mm'],
                             data['feed_mm_min'], data['gripper_settle_s'])
    return preview_plan(plan, data['start_mm'], checker)


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('JSON字段重复: ' + key)
        result[key] = value
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checker', required=True, help='主机构建的robotarm_plan_check可执行文件')
    parser.add_argument('--input', required=True, help='含明确起点及坐标的JSON；示例坐标不代表实际标定')
    args = parser.parse_args()
    try:
        data = json.loads(Path(args.input).read_text(encoding='utf-8-sig'), object_pairs_hook=unique_object)
        report = preview_request(data, args.checker)
    except (ValueError, OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        parser.exit(2, '离线预演失败: ' + str(exc) + '\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['model']['pass'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
