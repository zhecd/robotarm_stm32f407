"""上位机串口客户端。导入模块和默认 CLI 均不打开串口，不自动重发运动命令。"""
from __future__ import annotations
import argparse
import math
import time
from feedback_snapshot import parse_feedback
from gripper_snapshot import parse_gripper
from task_preflight import (PreflightReport, supported_identity, parse_runtime,
                            parse_limits, evaluate_observations)

class RobotArmError(RuntimeError):
    """固件拒绝、通信失步或执行状态不确定。"""

class RobotArmClient:
    """传入已打开的 transport（write/readline），一问一答；可替换为测试传输层。"""
    def __init__(self,transport,clock=time.monotonic):
        self.transport=transport
        self.clock=clock
        self.uncertain=False
        self._rx=bytearray()

    def request(self,command,expected=('ok','queued'),timeout=5.0):
        """只发送一次。超时或无法匹配回复后锁住运动发送，查询和停止仍可用。"""
        command=command.strip()
        if not command or len(command)>255 or any(ord(c)<32 or ord(c)>126 for c in command):
            raise ValueError('命令必须是单行 ASCII，最多 255 个字符')
        if not math.isfinite(timeout) or timeout<=0:
            raise ValueError('timeout 必须是正数')
        # 查询和显式停止/恢复可以在失步时发出；M999会动作，绝不是只读查询。
        allowed_when_uncertain=command in {'M114','M115','M119','M408','M409','M410','M112','M999'}
        allowed_when_uncertain=allowed_when_uncertain or command in {f'{query} N{node}' for query in ('M122','M123') for node in range(4)}
        if self.uncertain and not allowed_when_uncertain:
            raise RobotArmError('先查询状态并处理通信异常；必要时停止并重新回零，禁止盲目重发动作')
        payload=(command+'\r\n').encode('ascii')
        try:
            if self.transport.write(payload)!=len(payload):
                raise RobotArmError('串口仅写出部分命令，执行状态不确定')
            deadline=self.clock()+timeout
            recovery_started=False
            while self.clock()<deadline:
                if b'\n' not in self._rx:
                    self._rx.extend(self.transport.readline())
                if len(self._rx)>4096: raise RobotArmError('回复行过长，协议失步')
                if b'\n' not in self._rx: continue
                raw,_,remaining=self._rx.partition(b'\n')
                self._rx=bytearray(remaining)
                line=raw.decode('ascii').strip()
                if not line: continue
                if line.startswith('#'):
                    if 'UART TX queue overflow' in line: raise RobotArmError('固件发送队列溢出，回复可能丢失')
                    continue
                if line.startswith('error:'): raise RobotArmError(line)
                if command=='M999' and line=='M999: homing started':
                    recovery_started=True
                    continue
                if command=='M999' and line=='M999OK' and not recovery_started:
                    raise RobotArmError('未收到本轮回零开始确认，不能用孤立M999OK解除通信失步')
                if any(line==prefix or line.startswith(prefix+' ') for prefix in expected): return line
                raise RobotArmError('收到未匹配回复，执行状态不确定: '+line)
            raise TimeoutError('等待回复超时；未自动重发命令: '+command)
        except (OSError,UnicodeError,RobotArmError,TimeoutError):
            self.uncertain=True
            raise

    def status(self): return self.request('M408',('M408',))
    def capabilities(self): return self.request('M115',('M115',))
    def position(self): return self.request('M114',('M114',))
    def limits(self): return self.request('M119',('M119',))
    def preflight(self,max_duration_s=2.0):
        """六次只读查询；检查前后运行状态，不重试、不回零、不解除uncertain。

        调用方已打开传输对象。本结果供视觉任务展示阻塞原因，不能用来自动启动动作。
        不支持的固件只读M115即返回；格式错误则沿用会话失步保护并中止本轮。
        """
        if not math.isfinite(max_duration_s) or max_duration_s<=0:
            raise ValueError('max_duration_s 必须是正数')
        started=self.clock()
        identity=self.capabilities()
        try:
            supported,_=supported_identity(identity)
            if not supported:
                return PreflightReport(identity,started,self.clock(),('UNSUPPORTED_FIRMWARE',))
            before=parse_runtime(self.status())
            limits=parse_limits(self.limits())
            feedback_started=self.clock()
            feedback=self.feedback()
            gripper=self.gripper_status()
            after=parse_runtime(self.status())
            return evaluate_observations(identity,before,after,limits,feedback,gripper,
                uncertain=self.uncertain,started_at=started,finished_at=self.clock(),
                feedback_started_at=feedback_started,max_duration_s=max_duration_s)
        except ValueError as exc:
            self.uncertain=True
            raise RobotArmError('无效任务前状态: '+str(exc)) from exc
    def gripper_status(self):
        """M410软件快照；需GRIPPER_STATUS能力，不清uncertain、不触发动作。"""
        reply=self.request('M410',('M410',))
        try:
            return parse_gripper(reply)
        except ValueError as exc:
            self.uncertain=True
            raise RobotArmError('无效夹爪快照: '+str(exc)) from exc
    def feedback(self):
        """M409逐轴角度和年龄快照；需固件M115声明FEEDBACK_AGE能力。

        返回有效反馈不会清除uncertain，也不表示机械臂已回零/可动作。
        """
        reply=self.request('M409',('M409',))
        try:
            return parse_feedback(reply)
        except ValueError as exc:
            self.uncertain=True
            raise RobotArmError('无效反馈快照: '+str(exc)) from exc
    def driver_health(self,node):
        """仅WAVE7诊断支持；返回原始M123行，VALID:0含失败寄存器及原因，不自动重试。

        node是UART地址0..3；CS_CODE为电流档位编码，不能当成安培数。
        """
        if type(node) is not int or not 0<=node<=3:
            raise ValueError('TMC UART地址须为0..3的整数')
        return self.request(f'M123 N{node}',(f'M123 NODE:{node}',))
    def move_to(self,x_mm,y_mm,z_mm,feed_mm_min=1000):
        values=(x_mm,y_mm,z_mm,feed_mm_min)
        if any(not math.isfinite(v) for v in values) or not 1<=feed_mm_min<=3000:
            raise ValueError('坐标须为有限 mm 数值；进给速度范围为 (0,3000] mm/min')
        return self.request(f'G1 X{x_mm:.3f} Y{y_mm:.3f} Z{z_mm:.3f} F{int(feed_mm_min)}')
    def wait_motion(self,timeout=120):
        """等待规划与 STEP 队列完成；不把它当作末端实测到位或夹爪到位证明。"""
        return self.request('M400',('ok',),timeout)
    def open_gripper(self): return self.request('M3',('M3OK',))
    def close_gripper(self): return self.request('M5',('M5OK',))
    def stop(self):
        """软件停止；依赖串口和主循环响应，不能代替硬件急停。"""
        return self.request('M112',('M112OK',))
    def recover(self,timeout=240):
        """显式触发真实回零，须先确认现场。240秒覆盖220秒回零及检查开销。

        必须依次收到本轮开始与完成回复，且只发送一次M999。超时不自动重试。
        transport.readline须有有限超时；客户端时钟不能中断无限阻塞的外部传输。
        """
        result=self.request('M999',('M999OK',),timeout)
        self.uncertain=False
        return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',help='例如 COM3；仅指定端口不会连接')
    parser.add_argument('--connect',action='store_true',help='明确允许打开串口并发送只读查询')
    args=parser.parse_args()
    if not args.connect:
        print('离线模式：未打开串口。现场确认后可用 --port COM3 --connect 查询 M115/M408/M119。')
        return 0
    if not args.port: parser.error('--connect 需要 --port')
    import serial  # 仅实际连接时需要 pyserial；离线测试不依赖此包。
    port=serial.Serial(port=None,baudrate=115200,timeout=0.2,write_timeout=2)
    try:
        port.dtr=False; port.rts=False
        port.port=args.port; port.open()
        client=RobotArmClient(port)
        for query in (client.capabilities,client.status,client.limits): print(query())
    finally:
        port.close()
    return 0

if __name__=='__main__': raise SystemExit(main())
