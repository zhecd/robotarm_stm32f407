"""颜色分拣的离线任务编排示例：输入必须是已标定的机械臂坐标（mm）。

本模块不做颜色识别、相机标定或像素到机械臂坐标变换，不打开串口。
返回的计划供后续上位机集成；真实坐标、避障路径和夹爪等待时间须另行验证。
"""
from dataclasses import dataclass
import math

def _finite_number(value):
    if type(value) not in (int,float): return False
    try: return math.isfinite(value)
    except OverflowError: return False

@dataclass(frozen=True)
class Detection:
    color: str
    x_mm: float
    y_mm: float
    z_mm: float

def build_sorting_plan(detection:Detection,bins:dict,safe_z_mm:float,feed_mm_min:int=1000,gripper_settle_s:float=0.8):
    """bins 为 color -> (x_mm,y_mm,z_mm)；等待至少覆盖原600ms软件保持窗口。

    仅生成离线计划。几何可达性由复用真实C规划器的预演另查；不在Python复制机构参数。
    """
    if not isinstance(detection,Detection) or not isinstance(detection.color,str) or not detection.color:
        raise ValueError('检测结果须包含非空颜色标识和机械臂坐标')
    if detection.color not in bins: raise ValueError('未配置该颜色的放置位置')
    try: target=tuple(bins[detection.color])
    except TypeError as exc: raise ValueError('放置位置必须为3维坐标') from exc
    if len(target)!=3: raise ValueError('放置位置必须为 3 维 mm 坐标')
    values=(detection.x_mm,detection.y_mm,detection.z_mm,*target,safe_z_mm,gripper_settle_s)
    if any(not _finite_number(v) for v in values):
        raise ValueError('坐标与等待时间必须是有效有限值')
    if not 0.6<=gripper_settle_s<=60:
        raise ValueError('夹爪等待须为0.6..60秒，覆盖固件600ms保持窗口')
    if safe_z_mm<max(detection.z_mm,target[2]): raise ValueError('安全高度低于抓取或放置高度')
    if type(feed_mm_min) is not int or not 1<=feed_mm_min<=3000: raise ValueError('进给速度须为 1..3000 mm/min 的整数')
    plan=[]
    def move(x,y,z):
        plan.extend([('command',f'G1 X{x:.3f} Y{y:.3f} Z{z:.3f} F{feed_mm_min}'),('command','M400')])
    def grip(command): plan.extend([('command',command),('wait_seconds',gripper_settle_s)])
    # 先升高再水平移动。该路径仍需结合机构、工装和障碍物验证。
    plan.extend([('command',f'G1 Z{safe_z_mm:.3f} F{feed_mm_min}'),('command','M400')])
    grip('M3')
    move(detection.x_mm,detection.y_mm,safe_z_mm)
    move(detection.x_mm,detection.y_mm,detection.z_mm)
    grip('M5')
    move(detection.x_mm,detection.y_mm,safe_z_mm)
    move(target[0],target[1],safe_z_mm)
    move(*target)
    grip('M3')
    move(target[0],target[1],safe_z_mm)
    if any(kind=='command' and len(value)>255 for kind,value in plan):
        raise ValueError('坐标生成的命令超过串口255字符上限')
    return plan
