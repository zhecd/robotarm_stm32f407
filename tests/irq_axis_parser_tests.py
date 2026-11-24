"""解析真实C记录器导出的字节，覆盖环形顺序、冻结和拒绝不稳定快照。"""
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/startup_audit'))
from parse_axis_irq import decode_axes
from prepare_axis_capture import make_cfg, EXPECTED
EXE = Path(sys.argv.pop(1))


class AxisParserTests(unittest.TestCase):
    def test_capture_generator_is_symbol_bound_and_read_only(self):
        symbols={name:{'address':0x20001000+i*0x5000,'bytes':size} for i,(name,size) in enumerate(EXPECTED.items())}
        cfg=make_cfg(symbols,Path(tempfile.gettempdir())/'axis capture')
        self.assertIn('0x20001000 180',cfg)
        self.assertEqual(cfg.count('dump_image '),5)
        for forbidden in ['reset run','reset halt','mww ','write_memory ','flash write','resume','halt\n']:
            self.assertNotIn(forbidden,cfg)
        for key in EXPECTED:
            bad={k:dict(v) for k,v in symbols.items()};bad[key]['bytes']+=4
            with self.assertRaises(ValueError):make_cfg(bad,Path('capture'))
        bad={k:dict(v) for k,v in symbols.items()};bad['g_irq_axis_records']['address']=0x2001fff0
        with self.assertRaises(ValueError):make_cfg(bad,Path('capture'))
        with self.assertRaises(ValueError):make_cfg(symbols,Path('capture} unsafe'))

    def fixture(self, scenario):
        with tempfile.TemporaryDirectory(prefix='robotarm_axis_irq_') as directory:
            path = Path(directory)/'fixture.bin'
            subprocess.run([str(EXE),scenario,str(path)],check=True,capture_output=True)
            data = path.read_bytes()
        return data[:180],data[180:]

    def test_ring_real_C_bytes(self):
        h, r = self.fixture('ring'); d = decode_axes(h,r,h); a=d['axes'][1]
        self.assertEqual([x['axis_sequence'] for x in a['records']], list(range(1,33))+list(range(89,121)))
        self.assertEqual(a['tail_overwrites'],56)
        self.assertFalse(d['root_cause_determined'])

    def test_fairness_and_shared(self):
        h,r=self.fixture('fairness');d=decode_axes(h,r,h)
        self.assertEqual([a['events_total'] for a in d['axes']],[300,1,1])
        h,r=self.fixture('shared');d=decode_axes(h,r,h)
        for a in d['axes']:
            self.assertEqual(a['records'][-1]['vector_sequence'],80)
            self.assertEqual(a['records'][-1]['callback_mask'],7)

    def test_frozen_tail(self):
        h,r=self.fixture('freeze');a=decode_axes(h,r,h)['axes'][2]
        self.assertTrue(a['frozen']);self.assertEqual(a['ignored_after_freeze'],1)
        self.assertEqual(a['records'][-1]['axis_sequence'],256)
        self.assertEqual(a['freeze_vector'],256)

    def test_unstable_and_truncated_rejected(self):
        h,r=self.fixture('ring')
        with self.assertRaises(ValueError):decode_axes(h[:-4],r,h)
        with self.assertRaises(ValueError):decode_axes(h,r[:-4],h)
        odd=bytearray(h);struct.pack_into('<I',odd,60+56,241)
        with self.assertRaises(ValueError):decode_axes(odd,r,odd)
        changed=bytearray(h);changed[56]^=2
        with self.assertRaises(ValueError):decode_axes(h,r,changed)

    def test_corrupt_layout_counts_and_records_rejected(self):
        h,r=self.fixture('ring')
        for word,value in [(0,0),(1,2),(2,31),(4,80),(5,33),(7,32),(8,0),(9,0),(10,2)]:
            bad=bytearray(h);struct.pack_into('<I',bad,60+word*4,value)
            with self.subTest(word=word),self.assertRaises(ValueError):decode_axes(bad,r,bad)
        for offset,value in [(64*84,0),(64*84+4,100),(64*84+8+17*4,1)]:
            bad=bytearray(r);struct.pack_into('<I',bad,offset,value)
            with self.subTest(offset=offset),self.assertRaises(ValueError):decode_axes(h,bad,h)
        h,r=self.fixture('freeze');bad=bytearray(h);struct.pack_into('<I',bad,120+11*4,0x1c00)
        with self.assertRaises(ValueError):decode_axes(bad,r,bad)


if __name__=='__main__':unittest.main()
