"""用违规夹具验证检查器会失败，避免检查规则失效时形成假通过。"""
import pathlib
import sys
import tempfile
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'scripts'))
from architecture_rules import check_tree

class ArchitectureTests(unittest.TestCase):
    def check(self,files):
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp)
            for name,text in files.items():
                path=root/name; path.parent.mkdir(parents=True,exist_ok=True); path.write_text(text,encoding='utf-8')
            return check_tree(root)
    def test_cross_layer(self):
        errors=self.check({'RobotArm/Algorithm/a.c':'#include "device/dev.h"','RobotArm/Device/include/device/dev.h':''})
        self.assertTrue(any('Algorithm -> Device' in e for e in errors))
    def test_private(self):
        errors=self.check({'RobotArm/Service/command/c.c':'#include "ctrl.h"','RobotArm/Service/motion/internal/ctrl.h':''})
        self.assertTrue(any('private implementation' in e for e in errors))
    def test_transitive_public_hardware(self):
        errors=self.check({'RobotArm/Device/include/device/dev.h':'#include "wrapper.h"',
                           'RobotArm/Device/include/wrapper.h':'#include "usart.h"','Core/Inc/usart.h':'UART_HandleTypeDef port;'})
        self.assertTrue(any('device/dev.h: public header' in e for e in errors))
    def test_comments_and_strings(self):
        self.assertEqual(self.check({'RobotArm/Algorithm/a.c':'/* HAL_Foo */ const char *s="http://HAL_Foo";'}),[])
    def test_missing_root_fails(self): self.assertTrue(self.check({}))

if __name__=='__main__': unittest.main()
