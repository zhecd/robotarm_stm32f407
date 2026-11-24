"""M410纯解析及客户端离线测试；不构造真实串口。"""
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'tools/host'))
from gripper_snapshot import parse_gripper
from robotarm_client import RobotArmClient, RobotArmError

IDLE = 'M410 BOUND:0 TARGET:NONE ANGLE_DEG:0 PWM_REQUESTED:0 HOLD_MS:600 ELAPSED_MS:0 ERROR:0 POSITION:UNMEASURED'
OPEN = 'M410 BOUND:1 TARGET:OPEN ANGLE_DEG:100 PWM_REQUESTED:1 HOLD_MS:600 ELAPSED_MS:23 ERROR:0 POSITION:UNMEASURED'
DONE = OPEN.replace('PWM_REQUESTED:1', 'PWM_REQUESTED:0').replace('ELAPSED_MS:23', 'ELAPSED_MS:600')


class Transport:
    def __init__(self, replies):
        self.replies = list(replies)
        self.writes = []
        self.time = 0.0

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def readline(self):
        self.time += 0.1
        return self.replies.pop(0) if self.replies else b''


class GripperStatusTests(unittest.TestCase):
    def test_unbound_and_bound_without_target(self):
        for line in (IDLE, IDLE.replace('BOUND:0', 'BOUND:1')):
            s = parse_gripper(line)
            self.assertFalse(s.hold_finished)
            self.assertFalse(s.pwm_requested)

    def test_active_stopped_and_close_targets(self):
        self.assertFalse(parse_gripper(OPEN).hold_finished)
        self.assertTrue(parse_gripper(DONE).hold_finished)
        s = parse_gripper(DONE.replace('TARGET:OPEN', 'TARGET:CLOSE').replace('ANGLE_DEG:100', 'ANGLE_DEG:72'))
        self.assertEqual(s.target_angle_deg, 72)
        self.assertTrue(s.hold_finished)

    def test_early_stop_and_expired_but_not_stopped(self):
        self.assertFalse(parse_gripper(DONE.replace('ELAPSED_MS:600', 'ELAPSED_MS:20')).hold_finished)
        self.assertFalse(parse_gripper(OPEN.replace('ELAPSED_MS:23', 'ELAPSED_MS:600')).hold_finished)

    def test_errors_are_not_completion(self):
        for error in (1, 2):
            s = parse_gripper(DONE.replace('ERROR:0', f'ERROR:{error}'))
            self.assertFalse(s.hold_finished)
        failed = IDLE.replace('BOUND:0', 'BOUND:1').replace('ERROR:0', 'ERROR:1').replace('PWM_REQUESTED:0', 'PWM_REQUESTED:1')
        self.assertTrue(parse_gripper(failed).pwm_requested)
        self.assertFalse(parse_gripper(failed).hold_finished)

    def test_custom_angle_range(self):
        for angle in ('0', '90.5', '1.8e2'):
            s = parse_gripper(OPEN.replace('TARGET:OPEN', 'TARGET:CUSTOM').replace('ANGLE_DEG:100', 'ANGLE_DEG:' + angle))
            self.assertEqual(s.target, 'CUSTOM')

    def test_malformed_or_inconsistent_fields(self):
        cases = [IDLE + ' BOUND:0', IDLE + ' EXTRA:0', IDLE.replace(' ERROR:0', ''),
                 OPEN.replace('ANGLE_DEG:100', 'ANGLE_DEG:nan'), OPEN.replace('ANGLE_DEG:100', 'ANGLE_DEG:1e999'),
                 OPEN.replace('ANGLE_DEG:100', 'ANGLE_DEG:72'), OPEN.replace('TARGET:OPEN', 'TARGET:CLOSED'),
                 OPEN.replace('ELAPSED_MS:23', 'ELAPSED_MS:601'), OPEN.replace('HOLD_MS:600', 'HOLD_MS:601'),
                 OPEN.replace('ERROR:0', 'ERROR:3'), OPEN.replace('PWM_REQUESTED:1', 'PWM_REQUESTED:2'),
                 OPEN.replace('BOUND:1', 'BOUND:0'), OPEN.replace('POSITION:UNMEASURED', 'POSITION:ARRIVED'),
                 IDLE.replace('ELAPSED_MS:0', 'ELAPSED_MS:1'), IDLE.replace('ERROR:0', 'ERROR:1'),
                 IDLE.replace('BOUND:0', 'BOUND:1').replace('PWM_REQUESTED:0', 'PWM_REQUESTED:1'),
                 OPEN.replace('TARGET:OPEN', 'TARGET:CUSTOM').replace('ANGLE_DEG:100', 'ANGLE_DEG:181'),
                 OPEN.replace('TARGET:OPEN', 'TARGET:CUSTOM').replace('ANGLE_DEG:100', 'ANGLE_DEG:-1')]
        for line in cases:
            with self.subTest(line=line), self.assertRaises(ValueError):
                parse_gripper(line)

    def test_client_query_works_while_uncertain_and_never_unlocks(self):
        t = Transport([(DONE + '\r\n').encode('ascii')])
        client = RobotArmClient(t, clock=lambda: t.time)
        client.uncertain = True
        self.assertTrue(client.gripper_status().hold_finished)
        self.assertTrue(client.uncertain)
        self.assertEqual(t.writes, [b'M410\r\n'])

    def test_client_rejects_corrupt_reply_and_does_not_retry(self):
        t = Transport([(DONE + ' TARGET:OPEN\r\n').encode('ascii')])
        client = RobotArmClient(t, clock=lambda: t.time)
        with self.assertRaises(RobotArmError):
            client.gripper_status()
        self.assertTrue(client.uncertain)
        self.assertEqual(t.writes, [b'M410\r\n'])

    def test_client_timeout_is_single_send(self):
        t = Transport([])
        client = RobotArmClient(t, clock=lambda: t.time)
        with self.assertRaises(TimeoutError):
            client.gripper_status()
        self.assertTrue(client.uncertain)
        self.assertEqual(t.writes, [b'M410\r\n'])


if __name__ == '__main__':
    unittest.main()
