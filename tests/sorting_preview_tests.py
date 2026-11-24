"""生成计划并运行真实C业务链；被替换的只有设备、时钟和临界区端口。"""
import copy
import json
import math
import pathlib
import subprocess
import sys
import unittest
from unittest.mock import patch

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/host'))
from vision_sorting_plan import Detection, build_sorting_plan
from vision_sorting_preview import preview_plan, preview_request, unique_object

CHECKER = pathlib.Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else ROOT / 'build/HostTests/tests/robotarm_plan_check.exe'
EXAMPLE = json.loads((ROOT / 'tools/host/examples/red_sorting_preview.json').read_text(encoding='utf-8'))


class SortingPreviewTests(unittest.TestCase):
    def native(self, text, start=('0', '185', '240')):
        result = subprocess.run([str(CHECKER), *start], input=text, text=True,
                                capture_output=True, timeout=15)
        model = json.loads(result.stdout)
        self.assertEqual(result.returncode, 0 if model['pass'] else 2)
        self.assertTrue(model['simulation_only'])
        self.assertFalse(model['hardware_connected'])
        return model

    def test_red_plan_whole_real_chain(self):
        result = preview_request(EXAMPLE, CHECKER)
        model = result['model']
        self.assertTrue(model['pass'])
        self.assertEqual(model['completed_items'], 20)
        self.assertEqual(model['position_mm'], [20, 180, 160])
        self.assertEqual((model['fake_gripper_open'], model['fake_gripper_close']), (2, 1))
        self.assertTrue(all(steps > 0 for steps in model['fake_steps']))
        self.assertGreater(model['simulated_ms'], 2400)
        self.assertEqual(len(result['checker_sha256']), 64)

    def test_three_colors_use_supplied_bins(self):
        for color, target in [('red', (20, 180, 100)), ('green', (-30, 190, 120)), ('blue', (40, 190, 110))]:
            data = copy.deepcopy(EXAMPLE)
            data['detection']['color'] = color
            data['bins'] = {color: target}
            result = preview_request(data, CHECKER)
            self.assertTrue(result['model']['pass'])
            self.assertEqual(result['model']['position_mm'], [target[0], target[1], 160])

    def test_unreachable_pick_stops_before_grip_close(self):
        data = copy.deepcopy(EXAMPLE)
        data['detection']['x_mm'] = 1000
        model = preview_request(data, CHECKER)['model']
        self.assertFalse(model['pass'])
        self.assertEqual(model['reason'], 'MOVE_REJECTED')
        self.assertEqual(model['fake_gripper_close'], 0)
        self.assertLess(model['completed_items'], 20)

    def test_valid_endpoints_with_invalid_path_rejected_before_steps(self):
        # 起点/终点都可达；直线中部进入工具偏移内侧不可达区域。
        model = self.native('command\tG1 X120 Y20 Z105 F1000\ncommand\tM400\n', ('-120', '20', '105'))
        self.assertFalse(model['pass'])
        self.assertEqual(model['reason'], 'PATH_REJECTED')
        self.assertEqual(model['fake_steps'], [0, 0, 0])

    def test_soft_joint_limit_from_real_config(self):
        model = self.native('command\tG1 X0 Y-185 Z240 F1000\ncommand\tM400\n')
        self.assertFalse(model['pass'])
        self.assertEqual(model['fake_steps'], [0, 0, 0])

    def test_sub_point_one_mm_move_cannot_skip_joint_limit(self):
        radius=185
        start=(str(radius*math.sin(math.radians(96.99))), str(radius*math.cos(math.radians(96.99))), '240')
        target=(radius*math.sin(math.radians(97.01)), radius*math.cos(math.radians(97.01)))
        model=self.native(f'command\tG1 X{target[0]:.8f} Y{target[1]:.8f} Z240 F1000\ncommand\tM400\n',start)
        self.assertFalse(model['pass'])
        self.assertEqual(model['fake_steps'],[0,0,0])

    def test_native_contract_rejects_actions_outside_plan(self):
        for command in ('M999', 'M112', 'G0 X10', 'M115', 'M400', 'G1 F1000'):
            model = self.native('command\t' + command + '\n')
            self.assertFalse(model['pass'])
            self.assertEqual(model['fake_steps'], [0, 0, 0])

    def test_native_requires_motion_barrier_and_gripper_wait(self):
        for text in ['command\tG1 X1 F1000\ncommand\tM3\n',
                     'command\tM3\ncommand\tM5\n',
                     'command\tM3\nwait_seconds\t0.59\n',
                     'command\tM3\n', 'command\tG1 X1 F1000\n', '',
                     'command\tM3', 'wait_seconds\t0.8\n']:
            self.assertFalse(self.native(text)['pass'])

    def test_native_invalid_lines_and_start(self):
        for text in ['command\tG1 Xnan F1000\n', 'command\tG1 X1 F3001\n',
                     'command\tG1 X1 F0\n', 'bogus\tM3\n', 'command M3\n',
                     'command\tG1 X1 X2 F1000\n', 'command\t' + 'X' * 500 + '\n']:
            self.assertFalse(self.native(text)['pass'])
        for start in [('0', '185', '1000'), ('nan', '185', '240')]:
            self.assertFalse(self.native('command\tM3\nwait_seconds\t0.8\n', start)['pass'])

    def test_model_deadline_bounded(self):
        model = self.native('command\tM3\nwait_seconds\t60\ncommand\tM5\nwait_seconds\t60\n')
        self.assertFalse(model['pass'])
        self.assertEqual(model['reason'], 'MODEL_DEADLINE')
        self.assertLessEqual(model['simulated_ms'], 120000)

    def test_plan_numeric_validation_before_native_process(self):
        for invalid in (True, '100', None, float('nan'), float('inf'), 10**400):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    build_sorting_plan(Detection('red', invalid, 180, 100), {'red': (20, 180, 100)}, 160)
        for feed in (True, 0, 3001, 1000.5):
            with self.assertRaises(ValueError):
                build_sorting_plan(Detection('red', 10, 180, 100), {'red': (20, 180, 100)}, 160, feed)
        for wait in (True, 0, 0.599, 61):
            with self.assertRaises(ValueError):
                build_sorting_plan(Detection('red', 10, 180, 100), {'red': (20, 180, 100)}, 160, 1000, wait)

    def test_plan_format_and_safe_height(self):
        for target in (None, (1, 2), 'xyz'):
            with self.assertRaises(ValueError):
                build_sorting_plan(Detection('red', 10, 180, 100), {'red': target}, 160)
        with self.assertRaises(ValueError):
            build_sorting_plan(Detection('red', 10, 180, 100), {'red': (20, 180, 100)}, 99)
        with self.assertRaises(ValueError):
            build_sorting_plan(Detection('red', 1e300, 180, 100), {'red': (20, 180, 100)}, 160)

    def test_malformed_task_has_no_process_side_effect(self):
        for data in ({}, {**EXAMPLE, 'extra': 1}, {**EXAMPLE, 'schema': 'unknown'}):
            with patch('vision_sorting_preview.subprocess.run') as run:
                with self.assertRaises(ValueError): preview_request(data, CHECKER)
                run.assert_not_called()
        with self.assertRaises(ValueError):
            json.loads('{"schema":"x","schema":"y"}', object_pairs_hook=unique_object)

    def test_injected_lines_rejected_before_process(self):
        for plan in ([('command', 'M3\nM999')], [('command', 'M3\tM999')],
                     [('wait_seconds', float('nan'))], [('command', '')],
                     [('command', 'M3')] * 257, [('unknown', 1)]):
            with patch('vision_sorting_preview.subprocess.run') as run:
                with self.assertRaises(ValueError): preview_plan(plan, [0, 185, 240], CHECKER)
                run.assert_not_called()

    def test_preview_timeout_or_bad_output_is_not_success(self):
        with patch('vision_sorting_preview.subprocess.run', side_effect=subprocess.TimeoutExpired('model', 1)):
            with self.assertRaises(subprocess.TimeoutExpired): preview_plan([('command', 'M3')], [0, 185, 240], CHECKER)
        for output in ('partial', '{}', '{"pass":true,"simulation_only":false}'):
            with patch('vision_sorting_preview.subprocess.run', return_value=subprocess.CompletedProcess([], 0, output, '')):
                with self.assertRaises(RuntimeError): preview_plan([('command', 'M3')], [0, 185, 240], CHECKER)


if __name__ == '__main__':
    unittest.main()
