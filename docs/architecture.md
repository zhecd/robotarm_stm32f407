# 软件架构

本项目采用 `APP → SERVICE → DEVICE → DRIVER → BSP/HAL → Hardware` 的依赖方向。`os/` 为各层提供时间和延时等运行时适配，不承担机械臂业务逻辑。

```text
APP
  G-code、PS2 遥操作、模式管理、校准流程
      ↓
SERVICE
  回零、夹爪服务、运动学、轨迹规划、闭环与安全策略
      ↓
DEVICE
  关节、夹爪、限位、操作者输入
      ↓
DRIVER
  AS5600、TMC2209、STEP/DIR、舵机 PWM、PS2 协议
      ↓
BSP / HAL
  UART、DMA、I2C、TIM、PWM、GPIO 与 STM32 HAL
```

## 目录职责

| 目录 | 典型文件 | 职责 |
|---|---|---|
| `app/` | `app_gcode_exec`、`app_teleop` | 将外部输入组织为任务，不直接操作器件。 |
| `service/` | `svc_homing`、`svc_gripper` | 提供机械臂业务能力。 |
| `service/control/` | `ctrl_kinematics`、`ctrl_planner` | 承担运动学、规划、脉冲引擎和闭环算法。 |
| `device/` | `dev_joint`、`dev_gripper` | 将多个驱动组合为关节、夹爪等机械臂部件。 |
| `driver/` | `drv_as5600`、`drv_tmc2209` | 处理具体芯片协议、寄存器和时序。 |
| `bsp/` | `bsp_uart1`、`bsp_led` | 管理板级外设资源。 |
| `os/` | `os_adapter` | 屏蔽裸机与未来 RTOS 的时间和延时差异。 |
| `common/` | `robot_config`、`robot_home_pose` | 保存跨层的数据类型、参数与数学工具。 |

## 关键边界

- `ctrl_motion_engine` 只通过 `dev_joint` 输出方向和步进脉冲，不直接持有步进器对象。
- `ctrl_closed_loop` 只读取关节反馈与关节限位状态，不直接访问 AS5600 实例。
- `svc_homing` 调用关节和限位设备接口，不依赖 GPIO 引脚定义。
- `app_gcode_exec` 调用 Service/Control 接口，不依赖 UART、PWM、I2C 或具体芯片类型。

这种边界使 AS5600、TMC2209 或舵机被替换时，主要修改范围集中在 `driver/` 与 `device/`，而不影响 G-code、运动学和上位机协议。
