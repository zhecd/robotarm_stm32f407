"""测试生成器和独立Flash比对；合成ELF/临时文件，不连接设备。"""
from pathlib import Path
import hashlib,json,struct,sys,tempfile,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/startup_trial'))
from prepare_acceptance import identity,flash_segments,prepare
from verify_flash import verify

def synthetic_elf(hold=0):
    b=bytearray(512);b[:6]=b'\x7fELF\x01\x01';struct.pack_into('<I',b,28,52);struct.pack_into('<HH',b,42,32,1)
    token=identity(0,hold).encode();b.extend(token)
    struct.pack_into('<8I',b,52,1,512,0x08000000,0x08000000,len(token),len(token),5,4)
    return bytes(b)

class AcceptanceMaterials(unittest.TestCase):
    def firmware(self,root,hold=0):
        root.mkdir();names=['ProductionHoldLocked','ProductionHoldTrial'] if hold else ['ProductionLocked','ProductionTrial']
        for n in names:
            data=synthetic_elf(hold);(root/(n+'.elf')).write_bytes(data)
            (root/(n+'_audit.json')).write_text(json.dumps({'sha256':hashlib.sha256(data).hexdigest(),'compiled_motion':n.endswith('Trial')}))
    def test_exact_identity_profiles(self):
        self.assertIn('STARTUP8',identity(0,0));self.assertIn('STARTUP9',identity(1,10000))
        self.assertIn('READY_HOLD_MS:10000',identity(1,10000))
        for args in [(True,0),(2,0),(1,1)]:
            with self.assertRaises(ValueError):identity(*args)
    def test_no_device_defaults_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as td:
            r=Path(td);self.firmware(r/'firmware');out=r/'trial'
            prepare(out,r/'firmware');a=json.loads((out/'authorization.json').read_text())
            self.assertFalse(a['approved']);self.assertFalse((out/'deployment_authorized.claim').exists())
            self.assertFalse((out/'start_once.claim').exists())
            cfg=(out/'flash_locked.cfg').read_text();self.assertIn('DEPLOY_NOT_AUTHORIZED',cfg);self.assertIn('BACKUP_REQUIRED',cfg)
            self.assertIn('old_debug',cfg);self.assertIn('OUTPUTS_NOT_DISABLED',cfg)
            with self.assertRaises(FileExistsError):prepare(out,r/'firmware')
    def test_hold_profile_materials(self):
        with tempfile.TemporaryDirectory() as td:
            r=Path(td);self.firmware(r/'firmware',10000);out=r/'trial';prepare(out,r/'firmware',hold_ms=10000)
            self.assertEqual(json.loads((out/'authorization.json').read_text())['ready_hold_ms'],10000)
            self.assertIn('STARTUP9',(out/'serial_common.ps1').read_text())
    def test_wrong_hash_and_bad_port_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            r=Path(td);self.firmware(r/'firmware')
            with self.assertRaises(ValueError):prepare(r/'out',r/'firmware',port='COM4;START')
            (r/'firmware/ProductionLocked.elf').write_bytes(synthetic_elf()+b'changed')
            with self.assertRaises(ValueError):prepare(r/'out',r/'firmware')
            self.assertFalse((r/'out').exists())
    def test_independent_flash_comparison(self):
        with tempfile.TemporaryDirectory() as td:
            r=Path(td);e=synthetic_elf();(r/'f.elf').write_bytes(e);d=bytearray(524288)
            for a,o,n in flash_segments(e):d[a-0x08000000:a-0x08000000+n]=e[o:o+n]
            (r/'f.bin').write_bytes(d);self.assertTrue(verify(r/'f.elf',r/'f.bin')['pass'])
            d[0]^=1;(r/'f.bin').write_bytes(d)
            with self.assertRaises(ValueError):verify(r/'f.elf',r/'f.bin')
            (r/'f.bin').write_bytes(d[:32])
            with self.assertRaises(ValueError):verify(r/'f.elf',r/'f.bin')
    def test_bad_elf(self):
        for d in [b'',b'notelf',bytes(100)]:
            with self.assertRaises(ValueError):flash_segments(d)
if __name__=='__main__':unittest.main()
