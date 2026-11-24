"""任务前检查的真实客户端/纯解析测试：全程只用传输替身，不接设备。"""
from dataclasses import replace
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'tools/host'))
from robotarm_client import RobotArmClient, RobotArmError
from task_preflight import parse_runtime, parse_limits, evaluate_observations
from feedback_snapshot import parse_feedback
from gripper_snapshot import parse_gripper

IDENTITY = 'M115 FIRMWARE:RobotArm PROTOCOL:1 AXES:3 UART:115200 FEATURES:GCODE,STATUS,STOP,DIAGNOSTIC,IRQ_GUARD,FEEDBACK_AGE,GRIPPER_STATUS'
AUDIT = 'M115 FIRMWARE:RobotArmStartupAudit PROTOCOL:AUDIT1 AXES:3 UART:115200 MOTOR_ENABLE:LOCKED AUTO_MOTION:0 FEATURES:FEEDBACK_AGE,GRIPPER_STATUS,IRQ_EVIDENCE'
READY = 'M408 STATE:READY HOMED:1 ALLOWED:1 MODE:GCODE FAULT:0 MOTION_FAULT:0 QUEUE:0 PENDING:0 RECOVERY:0 IRQ_LATCH_MASK:0'
LOCKED = READY.replace('STATE:READY HOMED:1 ALLOWED:1', 'STATE:OUTPUT_LOCKED HOMED:0 ALLOWED:0')
LIMITS = 'M119 M1:OPEN M2:OPEN M3:OPEN'
FEEDBACK = 'M409 GENERATION:1 NOW_MS:100 FRAME:JOINT PRESENT_MASK:7 VALID_MASK:7 FRESH_MASK:7 MAX_AGE_MS:100 AGE_MS:1,2,3 READ_MS:1,1,1 MOTOR:10,20,30 JOINT:1,2,3'
GRIPPER = 'M410 BOUND:1 TARGET:CLOSE ANGLE_DEG:72 PWM_REQUESTED:0 HOLD_MS:600 ELAPSED_MS:600 ERROR:0 POSITION:UNMEASURED'
QUERIES = [b'M115\r\n', b'M408\r\n', b'M119\r\n', b'M409\r\n', b'M410\r\n', b'M408\r\n']


class Transport:
    def __init__(self, replies, delay=0.005):
        self.replies = [(x + '\r\n').encode('ascii') for x in replies]
        self.writes = []
        self.time = 0.0
        self.delay = delay

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def readline(self):
        self.time += self.delay
        return self.replies.pop(0) if self.replies else b''


class PreflightTests(unittest.TestCase):
    def client(self, replies=None, delay=0.005):
        t = Transport(replies if replies is not None else
                      [IDENTITY, READY, LIMITS, FEEDBACK, GRIPPER, READY], delay)
        return RobotArmClient(t, clock=lambda: t.time), t

    def evaluate(self, **changes):
        args = dict(identity=IDENTITY, before=parse_runtime(READY), after=parse_runtime(READY),
                    limit_mask=0, feedback=parse_feedback(FEEDBACK), gripper=parse_gripper(GRIPPER),
                    uncertain=False, started_at=0.0, feedback_started_at=0.01,
                    finished_at=0.03, max_duration_s=2.0)
        args.update(changes)
        return evaluate_observations(**args)

    def test_normal_readonly_sequence(self):
        client, t = self.client()
        report = client.preflight()
        self.assertTrue(report.observations_ready)
        self.assertEqual(report.blockers, ())
        self.assertEqual(t.writes, QUERIES)
        self.assertFalse(client.uncertain)

    def test_current_audit_is_not_task_ready(self):
        client, t = self.client([AUDIT, LOCKED, LIMITS,
                                FEEDBACK.replace('FRAME:JOINT', 'FRAME:UNAVAILABLE'),
                                GRIPPER, LOCKED])
        report = client.preflight()
        self.assertIn('OUTPUT_LOCKED_FIRMWARE', report.blockers)
        self.assertIn('NOT_HOMED_READY', report.blockers)
        self.assertIn('JOINT_FEEDBACK_NOT_FRESH', report.blockers)
        self.assertEqual(t.writes, QUERIES)

    def test_unknown_diagnostic_and_missing_capability_stop_after_identity(self):
        for identity in [IDENTITY.replace('RobotArm ', 'RobotArmDiagnostics '),
                         IDENTITY.replace('PROTOCOL:1', 'PROTOCOL:2'),
                         IDENTITY.replace('UART:115200', 'UART:9600'),
                         IDENTITY.replace('AXES:3', 'AXES:4'),
                         IDENTITY.replace(',GRIPPER_STATUS', ''),
                         IDENTITY.replace('FEEDBACK_AGE', 'XFEEDBACK_AGE'),
                         IDENTITY + ' MOTOR_ENABLE:LOCKED']:
            with self.subTest(identity=identity):
                client, t = self.client([identity])
                self.assertEqual(client.preflight().blockers, ('UNSUPPORTED_FIRMWARE',))
                self.assertEqual(t.writes, [QUERIES[0]])

    def test_uncertain_never_cleared_and_never_recovers(self):
        client, t = self.client()
        client.uncertain = True
        self.assertIn('COMMUNICATION_UNCERTAIN', client.preflight().blockers)
        self.assertTrue(client.uncertain)
        self.assertEqual(t.writes, QUERIES)

    def test_status_change_and_faults(self):
        for key, value, reason in [('state', 'HOMING', 'NOT_HOMED_READY'),
                                   ('homed', False, 'NOT_HOMED_READY'),
                                   ('allowed', False, 'NOT_HOMED_READY'),
                                   ('mode', 'PS2', 'NOT_GCODE_MODE'),
                                   ('fault', 8, 'FAULT_OR_IRQ_LATCH'),
                                   ('motion_fault', 7, 'FAULT_OR_IRQ_LATCH'),
                                   ('irq_latch_mask', 4, 'FAULT_OR_IRQ_LATCH'),
                                   ('queue', 1, 'BUSY_OR_RECOVERING'),
                                   ('pending', 1, 'BUSY_OR_RECOVERING'),
                                   ('recovery', True, 'BUSY_OR_RECOVERING')]:
            with self.subTest(key=key):
                changed = replace(parse_runtime(READY), **{key: value})
                report = self.evaluate(after=changed)
                self.assertIn('RUNTIME_CHANGED', report.blockers)
                self.assertIn(reason, report.blockers)

    def test_triggered_limit_each_axis(self):
        for axis in range(3):
            mask = parse_limits(LIMITS.replace(f'M{axis + 1}:OPEN', f'M{axis + 1}:TRIGGERED'))
            self.assertEqual(mask, 1 << axis)
            self.assertIn('LIMIT_TRIGGERED', self.evaluate(limit_mask=mask).blockers)

    def test_transport_elapsed_invalidates_previously_fresh_feedback(self):
        client, t = self.client(delay=0.04)
        report = client.preflight()
        self.assertIn('JOINT_FEEDBACK_NOT_FRESH', report.blockers)
        self.assertLess(report.finished_at, 2.0)
        self.assertEqual(t.writes, QUERIES)

    def test_feedback_age_boundary_and_raw_frame(self):
        feedback = replace(parse_feedback(FEEDBACK), age_ms=(80, 80, 80))
        self.assertIn('JOINT_FEEDBACK_NOT_FRESH', self.evaluate(
            feedback=feedback, feedback_started_at=0.0, finished_at=0.02).blockers)
        self.assertTrue(self.evaluate(feedback=feedback, feedback_started_at=0.0,
                                      finished_at=0.019).observations_ready)
        for frame in ('RAW', 'UNAVAILABLE'):
            self.assertIn('JOINT_FEEDBACK_NOT_FRESH', self.evaluate(
                feedback=replace(feedback, frame=frame)).blockers)

    def test_gripper_unbound_active_failed_and_early_stop(self):
        done = parse_gripper(GRIPPER)
        for changed in [replace(done, bound=False), replace(done, pwm_requested=True),
                        replace(done, error=1), replace(done, elapsed_ms=50),
                        replace(done, target='NONE')]:
            self.assertIn('GRIPPER_UNAVAILABLE_OR_ACTIVE', self.evaluate(gripper=changed).blockers)

    def test_whole_observation_window_and_invalid_clock(self):
        self.assertIn('OBSERVATION_WINDOW_EXPIRED', self.evaluate(finished_at=2.0).blockers)
        for changes in [dict(finished_at=-1.0), dict(feedback_started_at=1.0),
                        dict(max_duration_s=0), dict(finished_at=float('nan'))]:
            with self.assertRaises(ValueError): self.evaluate(**changes)

    def test_parsers_reject_missing_duplicate_unknown_and_invalid_fields(self):
        for line in [READY + ' HOMED:1', READY.replace(' FAULT:0', ''),
                     READY + ' EXTRA:1', READY.replace('HOMED:1', 'HOMED:2'),
                     READY.replace('QUEUE:0', 'QUEUE:-1'), READY.replace('PENDING:0', 'PENDING:1.0'),
                     READY.replace('IRQ_LATCH_MASK:0', 'IRQ_LATCH_MASK:8')]:
            with self.assertRaises(ValueError): parse_runtime(line)
        for line in [LIMITS + ' M3:OPEN', 'M119 M1:OPEN M2:OPEN',
                     LIMITS.replace('M3:OPEN', 'M3:CLOSED'), LIMITS + ' M4:OPEN']:
            with self.assertRaises(ValueError): parse_limits(line)

    def test_malformed_reply_aborts_and_latches_uncertain(self):
        client, t = self.client([IDENTITY, READY + ' HOMED:1'])
        with self.assertRaises(RobotArmError): client.preflight()
        self.assertTrue(client.uncertain)
        self.assertEqual(t.writes, QUERIES[:2])

    def test_timeout_no_retry_no_recovery(self):
        client, t = self.client([])
        with self.assertRaises(TimeoutError): client.preflight()
        self.assertEqual(t.writes, [QUERIES[0]])
        self.assertTrue(client.uncertain)

    def test_malformed_identity_does_not_send_followup(self):
        client, t = self.client([IDENTITY + ' UART:115200'])
        with self.assertRaises(RobotArmError): client.preflight()
        self.assertEqual(t.writes, [QUERIES[0]])

    def test_invalid_duration_sends_nothing(self):
        client, t = self.client()
        for duration in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError): client.preflight(duration)
        self.assertEqual(t.writes, [])


if __name__ == '__main__':
    unittest.main()
