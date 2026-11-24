"""确认默认调用/未确认方向时，在创建串口和记录文件前拒绝运行。"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'tools/m1_homing'
POWERSHELL=shutil.which('pwsh') or shutil.which('powershell')


@unittest.skipUnless(POWERSHELL,'PowerShell is required for the HOME host entry')
class DefaultGuardTests(unittest.TestCase):
    def test_invalid_execution_requests_do_not_open_serial_or_write_attempt_files(self):
        text=(SOURCE/'execute_axis_home_after_approval.ps1').read_text(encoding='utf-8')
        # 如果未来移动了门禁位置，先拒绝测试，不能让故障注入接触真实串口。
        self.assertLess(text.index('Assert-AxisHomeRequest $ExecuteAuthorized.IsPresent'),text.index('$p=[IO.Ports.SerialPort]::new'))
        cases=[([], 'Explicit single-axis HOME approval required'),
               (['-ExecuteAuthorized'],'Explicit axis 1..3 required'),
               (['-ExecuteAuthorized','-Axis','2'],'Verified mechanical seek direction'),
               (['-ExecuteAuthorized','-Axis','2','-SeekDirection','D0','-DirectionEvidence','test data'],'Verified mechanical seek direction'),
               (['-ExecuteAuthorized','-Axis','3','-SeekDirection','D1','-DirectionVerified'],'Verified mechanical seek direction')]
        with tempfile.TemporaryDirectory(prefix='axis_home_no_serial_') as temp:
            folder=Path(temp)
            for name in ('execute_axis_home_after_approval.ps1','axis_home_contract.ps1'):
                shutil.copy2(SOURCE/name,folder/name)
            initial={p.name for p in folder.iterdir()}
            for args,expected in cases:
                with self.subTest(args=args):
                    process=subprocess.run([POWERSHELL,'-NoProfile','-NonInteractive','-File',str(folder/'execute_axis_home_after_approval.ps1'),*args],capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=10)
                    self.assertNotEqual(process.returncode,0)
                    self.assertIn(expected,process.stdout+process.stderr)
                    self.assertEqual(initial,{p.name for p in folder.iterdir()})


if __name__=='__main__':unittest.main()
