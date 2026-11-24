"""串口替身测试：分片、日志、错误、超时、禁止重发和离线分拣计划。"""
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'tools/host'))
from robotarm_client import RobotArmClient,RobotArmError
from vision_sorting_plan import Detection,build_sorting_plan
from feedback_snapshot import parse_feedback

class Transport:
    def __init__(self,rx): self.rx=list(rx); self.writes=[]; self.time=0
    def write(self,data): self.writes.append(data); return len(data)
    def readline(self):
        self.time+=0.1
        return self.rx.pop(0) if self.rx else b''

class ClientTests(unittest.TestCase):
    FEEDBACK='M409 GENERATION:1 NOW_MS:100 FRAME:JOINT PRESENT_MASK:7 VALID_MASK:7 FRESH_MASK:7 MAX_AGE_MS:100 AGE_MS:1,2,3 READ_MS:1,1,1 MOTOR:10,20,30 JOINT:1,2,3'
    def client(self,rx):
        transport=Transport(rx)
        return RobotArmClient(transport,clock=lambda:transport.time),transport
    def test_fragmented_reply_and_log(self):
        client,t=self.client([b'# log\r\n',b'M408 STATE:',b'READY\r\n'])
        self.assertEqual(client.status(),'M408 STATE:READY')
        self.assertEqual(t.writes,[b'M408\r\n'])
    def test_move_acceptance_and_completion_are_separate(self):
        client,t=self.client([b'queued\r\n',b'ok\r\n'])
        self.assertEqual(client.move_to(1,185,240),'queued')
        self.assertEqual(client.wait_motion(),'ok')
        self.assertEqual(t.writes[-1],b'M400\r\n')
    def test_timeout_no_retry(self):
        client,t=self.client([])
        with self.assertRaises(TimeoutError): client.request('G1 X1',timeout=0.2)
        self.assertEqual(len(t.writes),1)
        with self.assertRaises(RobotArmError): client.move_to(2,185,240)
        self.assertEqual(len(t.writes),1)
    def test_safety_error(self):
        client,_=self.client([b'error: M400 aborted by safety fault\r\n'])
        with self.assertRaises(RobotArmError): client.wait_motion()
        self.assertTrue(client.uncertain)
    def test_recovery_progress(self):
        client,t=self.client([b'M999: homing started\r\n',b'# homing\r\n',b'M999OK\r\n'])
        client.uncertain=True
        self.assertEqual(client.recover(),'M999OK')
        self.assertFalse(client.uncertain)
        self.assertEqual(len(t.writes),1)
    def test_recovery_does_not_accept_orphan_completion(self):
        client,t=self.client([b'M999OK\r\n']);client.uncertain=True
        with self.assertRaises(RobotArmError):client.recover()
        self.assertTrue(client.uncertain);self.assertEqual(t.writes,[b'M999\r\n'])
    def test_recovery_default_covers_220_second_homing(self):
        class SlowRecovery(Transport):
            def readline(self):
                self.time+=1
                if self.time==1:return b'M999: homing started\r\n'
                if self.time==221:return b'M999OK\r\n'
                return b''
        t=SlowRecovery([]);client=RobotArmClient(t,clock=lambda:t.time);client.uncertain=True
        self.assertEqual(client.recover(),'M999OK');self.assertFalse(client.uncertain)
        self.assertEqual(t.writes,[b'M999\r\n'])
    def test_recovery_timeout_does_not_resend_or_unlock(self):
        client,t=self.client([b'M999: homing started\r\n']);client.uncertain=True
        with self.assertRaises(TimeoutError):client.recover(timeout=0.3)
        self.assertTrue(client.uncertain);self.assertEqual(t.writes,[b'M999\r\n'])
    def test_malformed_command_and_unmatched_reply(self):
        client,t=self.client([b'M3OK\r\n'])
        with self.assertRaises(ValueError): client.request('G1 X1\nM3')
        self.assertEqual(t.writes,[])
        with self.assertRaises(RobotArmError): client.status()
    def test_sorting_plan_is_offline(self):
        plan=build_sorting_plan(Detection('red',10,180,100),{'red':(20,180,100)},160)
        self.assertEqual(plan[0][1],'G1 Z160.000 F1000')
        self.assertIn(('command','M5'),plan)
        for index,item in enumerate(plan):
            if item[0]=='command' and item[1].startswith('G1'):
                self.assertEqual(plan[index+1],('command','M400'))
        with self.assertRaises(ValueError): build_sorting_plan(Detection('blue',0,0,0),{},160)
    def test_driver_health_allowed_while_motion_uncertain(self):
        client,t=self.client([b'M123 NODE:0 VALID:0 REG:06 ERROR:4\r\n'])
        client.uncertain=True
        self.assertEqual(client.driver_health(0),'M123 NODE:0 VALID:0 REG:06 ERROR:4')
        self.assertTrue(client.uncertain)
        self.assertEqual(t.writes,[b'M123 N0\r\n'])
        for node in (-1,4,True,0.5,'0'):
            with self.assertRaises(ValueError):client.driver_health(node)
        self.assertEqual(len(t.writes),1)
    def test_driver_health_wrong_node_locks_session(self):
        client,t=self.client([b'M123 NODE:1 VALID:1\r\n'])
        with self.assertRaises(RobotArmError):client.driver_health(0)
        self.assertTrue(client.uncertain)
        self.assertEqual(t.writes,[b'M123 N0\r\n'])
    def test_feedback_query_remains_readonly_and_does_not_clear_uncertain(self):
        client,t=self.client([(self.FEEDBACK+'\r\n').encode('ascii')]);client.uncertain=True
        snapshot=client.feedback()
        self.assertEqual(snapshot.age_ms,(1,2,3));self.assertTrue(snapshot.joint_data_fresh)
        self.assertTrue(client.uncertain);self.assertEqual(t.writes,[b'M409\r\n'])
    def test_feedback_frames_and_stale_axes(self):
        self.assertFalse(parse_feedback(self.FEEDBACK.replace('FRAME:JOINT','FRAME:RAW')).joint_data_fresh)
        s=parse_feedback(self.FEEDBACK.replace('FRESH_MASK:7','FRESH_MASK:6').replace('AGE_MS:1,2,3','AGE_MS:100,2,3'))
        self.assertFalse(s.joint_data_fresh)
    def test_feedback_rejects_inconsistent_or_malformed_data(self):
        cases=[self.FEEDBACK+' NOW_MS:1',self.FEEDBACK.replace('FRESH_MASK:7','FRESH_MASK:8'),
               self.FEEDBACK.replace('VALID_MASK:7','VALID_MASK:6'),
               self.FEEDBACK.replace('AGE_MS:1,2,3','AGE_MS:100,2,3'),
               self.FEEDBACK.replace('MOTOR:10,20,30','MOTOR:nan,20,30'),
               self.FEEDBACK.replace('JOINT:1,2,3','JOINT:1e999,2,3'),
               self.FEEDBACK.replace('READ_MS:1,1,1','READ_MS:1,1'),
               self.FEEDBACK.replace('PRESENT_MASK:7','PRESENT_MASK:0'),
               self.FEEDBACK.replace('FRAME:JOINT','FRAME:WORLD'),
               self.FEEDBACK.replace('MAX_AGE_MS:100','MAX_AGE_MS:0')]
        for line in cases:
            with self.subTest(line=line):
                with self.assertRaises(ValueError):parse_feedback(line)
        client,_=self.client([(cases[0]+'\r\n').encode('ascii')])
        with self.assertRaises(RobotArmError):client.feedback()
        self.assertTrue(client.uncertain)

if __name__=='__main__': unittest.main()
