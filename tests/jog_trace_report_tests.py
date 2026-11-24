"""对离线报告的误判边界做故障注入，不连接真实串口。"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools/m1_homing'))
from jog_trace_report import TraceError, analyze, active_step_trend


def fixture():
    identity = ('M115 FIRMWARE:AxisHomingPrepare PROTOCOL:AXISHOME1 AXIS:2 UART:115200 '
                'MOTION_COMPILED:1 AUTO_MOTION:0 JOG_STEPS:20 JOG_DEADLINE_MS:500 '
                'SEEK_STEPS:200 BACKOFF_STEPS:40 HOME_DEADLINE_MS:5500 STEP_PERIOD_MS:20')
    done = ('M408 STATE:5 REASON:0 USED:1 EN:DISABLED STEP:0 JOG:20 SEEK:0 BACKOFF:0 '
            'DRIVER:1 DIR_CONFIRMED:0 HOMED:0 ALLOWED:0 RXERR:0 RXOVF:0')
    record = lambda ms,kind,text:dict(ms=ms,type=kind,text=text)
    return dict(axis=2,pass_=False,action_issued=True,error=None,final_reply=done,
                records=[record(0,'RX',identity),record(1,'TX','CONFIG'),
                         record(2,'RX','CONFIG VALID:1 RESULT:0 REG:FF READ_ERROR:0 CURRENT_REF:INTERNAL IRUN_CODE:8 IHOLD_CODE:4 GCONF:000001C0 CHOPCONF:14000053 IFCNT:254,2'),
                         record(3,'RX','M119 M1:OPEN M2:OPEN M3:OPEN'),
                         record(20,'TX','JOG D0'),record(420,'RX',done),
                         record(450,'RX','M122 LOCAL_REF:0 VALID:0 REASON:1 ORIGIN_MOTOR:0.000 REL_MOTOR:0.000 REL_JOINT:0.000 GEAR_RATIO:4.500 AGE_MS:0 WHOLE_HOMED:0 ALLOWED:0')],
                samples=[dict(ms=10,phase='pre',motor_deg=720.,age_ms=0),
                         dict(ms=40,phase='active',motor_deg=721.,age_ms=19),
                         dict(ms=430,phase='post',motor_deg=722.,age_ms=1),
                         dict(ms=450,phase='post',motor_deg=722.,age_ms=0)])


class ReportTests(unittest.TestCase):
    @staticmethod
    def step_fixture(angles):
        samples=[dict(ms=0,phase='pre',motor_deg=100.,age_ms=0)]
        records=[]
        for i,(count,angle) in enumerate(zip((4,7,10,13,16,18),angles)):
            samples.append(dict(ms=30+i*30,phase='active',motor_deg=angle,age_ms=19))
            records.append(dict(ms=31+i*30,type='RX',text=f'M408 STATE:1 REASON:0 USED:1 EN:ENABLED STEP:0 JOG:{count} SEEK:0 BACKOFF:0 DRIVER:1 DIR_CONFIRMED:0 HOMED:0 ALLOWED:0 RXERR:0 RXOVF:0'))
        samples.append(dict(ms=900,phase='post',motor_deg=-100.,age_ms=0))
        return samples,records

    def test_active_direction_ignores_enable_and_disable_offsets(self):
        for sign in (-1,1):
            samples,records=self.step_fixture([sign*x for x in (10,10.2,10.4,10.6,10.8,11.)])
            report=active_step_trend(samples,records)
            self.assertEqual(report['supported_encoder_sign'],sign)
        self.assertLess(samples[-1]['motor_deg']-samples[0]['motor_deg'],0)

    def test_active_direction_requires_enabled_fresh_consistent_steps(self):
        for angles in ([10]*6,[10,11,9,12,10,13]):
            self.assertEqual(active_step_trend(*self.step_fixture(angles))['supported_encoder_sign'],0)
        samples,records=self.step_fixture((10,11,12,13,14,15))
        for sample in samples:sample['age_ms']=26
        self.assertEqual(active_step_trend(samples,records)['supported_encoder_sign'],0)
        samples,records=self.step_fixture((10,11,12,13,14,15))
        for record in records:record['text']=record['text'].replace('EN:ENABLED','EN:DISABLED')
        self.assertEqual(active_step_trend(samples,records)['supported_encoder_sign'],0)

    def test_control_and_movement_do_not_grant_mechanical_acceptance(self):
        report=analyze(fixture())
        self.assertTrue(report['control_contract_passed'])
        self.assertTrue(report['movement_evidence_passed'])
        for key in ('mechanical_direction_verified','angle_accuracy_verified','home_verified','next_action_authorized','other_axes_physical_motion_verified'):
            self.assertFalse(report[key])

    def test_signed_multiturn_feedback(self):
        data=fixture()
        for item in data['samples']:item['motor_deg']=1440.-item['motor_deg']
        self.assertEqual(analyze(data)['peak_delta_motor_deg'],-2.)

    def test_one_d1_command_and_declared_direction(self):
        data=fixture();data['records'][4]['text']='JOG D1';data['jog_direction']='D1'
        report=analyze(data)
        self.assertTrue(report['control_contract_passed'])
        self.assertEqual(report['jog_direction'],'D1')
        data['jog_direction']='D0'
        self.assertFalse(analyze(data)['control_contract_passed'])

    def test_missing_or_small_movement_is_not_evidence(self):
        data=fixture()
        for item in data['samples']:item['motor_deg']=720.1
        self.assertFalse(analyze(data)['movement_evidence_passed'])

    def test_stale_age_boundary_100_rejected(self):
        data=fixture(); data['samples'][1]['age_ms']=100
        self.assertFalse(analyze(data)['control_contract_passed'])
        self.assertFalse(analyze(data)['movement_evidence_passed'])

    def test_wrong_axis_or_large_profile_identity(self):
        for old,new in [('AXIS:2','AXIS:3'),('JOG_STEPS:20','JOG_STEPS:400')]:
            data=fixture();data['records'][0]['text']=data['records'][0]['text'].replace(old,new)
            self.assertFalse(analyze(data)['control_contract_passed'])

    def test_duplicate_jog_and_unapproved_home_rejected(self):
        for command in ('JOG D0','HOME','JOG D1','M112'):
            data=fixture();data['records'].append(dict(ms=500,type='TX',text=command))
            self.assertFalse(analyze(data)['control_contract_passed'])

    def test_completion_after_host_bound_is_failure(self):
        data=fixture();data['records'][5]['ms']=671
        data['records'][6]['ms']=700
        data['samples'][2]['ms']=680;data['samples'][3]['ms']=700
        self.assertFalse(analyze(data)['control_contract_passed'])

    def test_enabled_final_state_is_failure(self):
        data=fixture();data['final_reply']=data['final_reply'].replace('EN:DISABLED','EN:ENABLED')
        self.assertFalse(analyze(data)['control_contract_passed'])

    def test_fault_then_done_cannot_hide_first_fault(self):
        data=fixture()
        data['records'].insert(5,dict(ms=100,type='RX',text=data['final_reply'].replace('STATE:5','STATE:7').replace('REASON:0','REASON:2')))
        self.assertFalse(analyze(data)['control_contract_passed'])

    def test_nonfinite_or_invalid_phases_raise(self):
        for invalid in (float('nan'),float('inf'),True):
            data=fixture();data['samples'][1]['motor_deg']=invalid
            with self.assertRaises(TraceError):analyze(data)
        data=fixture();data['samples'][1]['phase']='post'
        with self.assertRaises(TraceError):analyze(data)

    def test_unordered_time_and_negative_age_raise(self):
        for name,value in [('ms',9),('age_ms',-1)]:
            data=fixture();data['samples'][1][name]=value
            with self.assertRaises(TraceError):analyze(data)

    def test_opposing_initial_response_and_post_drift_are_visible(self):
        data=fixture();data['samples'][1]['motor_deg']=719.
        data['samples'][-1]['motor_deg']=721.5
        report=analyze(data)
        self.assertIn('initial_response_opposes_largest_excursion',report['warnings'])
        self.assertIn('feedback_excursions_on_both_sides_of_baseline',report['warnings'])
        self.assertIn('post_disable_variation_over_0_2_motor_deg',report['warnings'])
        self.assertFalse(report['mechanical_direction_verified'])

    def test_ifcnt_bad_increment_and_restored_reference_rejected(self):
        data=fixture();data['records'][2]['text']=data['records'][2]['text'].replace('254,2','254,3')
        self.assertFalse(analyze(data)['control_contract_passed'])
        data=fixture();data['records'][-1]['text']=data['records'][-1]['text'].replace('LOCAL_REF:0','LOCAL_REF:2')
        self.assertFalse(analyze(data)['control_contract_passed'])


if __name__=='__main__':unittest.main()
