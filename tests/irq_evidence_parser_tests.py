import struct
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/startup_audit'))
from parse_irq_evidence import decode


class EvidenceTests(unittest.TestCase):
    def fixture(self):
        h = struct.pack('<9I', 0x49525132, 64, 76, 1, 1, 0, 1, 1, 1)
        w = [10, 0xfffffffe, 0x1c00, 0x1c00, 0x1c00, 0x2200, 2, 0, 0x1c00, 0, 56,
             0x1c00, 0x1800, 0x1000, 0x1800, 0x1000, 0, 7, 4]
        return h, bytearray(struct.pack('<19I', *w) + bytes(63*76))

    def test_positive(self):
        h, b = self.fixture(); r = decode(h, b)['records'][0]
        self.assertEqual(r['cycles_elapsed'], 6)
        self.assertTrue(r['mapped_to_PC10_PC11_PC12'])
        self.assertEqual(r['active_limit_mask'], 0)
        self.assertEqual(r['callback_without_pending_at_call'], [])
        self.assertEqual(r['pending_after_handler'], [])

    def test_facts_not_diagnosis(self):
        h, b = self.fixture()
        struct.pack_into('<I', b, 11*4, 0)  # M1 callback但调用前没有该路pending。
        struct.pack_into('<I', b, 15*4, 0x800)
        d = decode(h, b); r = d['records'][0]
        self.assertEqual(r['callback_without_pending_at_call'], [1])
        self.assertEqual(r['pending_after_handler'], [2])
        self.assertFalse(d['root_cause_determined'])

    def test_malformed(self):
        h, b = self.fixture()
        for badh, badb in [(h[:-1], b), (h, b[:-4]), (bytes(36), b),
                           (struct.pack('<9I', 0x49525132, 64, 76, 65, 65, 0, 0, 0, 0), b)]:
            with self.assertRaises(ValueError): decode(badh, badb)


if __name__ == '__main__': unittest.main()
