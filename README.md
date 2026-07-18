# robotarm_stm32f407

基于 STM32F407 的 3 自由度教学机械臂下位机固件。项目采用裸机超级循环与中断调度，提供 UART G-code 控制、PS2 手柄遥操作、限位保护、回零、AS5600 编码器反馈和夹爪控制。

该项目面向具有 STM32 与 C 语言基础、正在学习机器人系统软件组织方式的读者。运动学、轨迹规划与硬件驱动分别位于不同模块中，便于阅读、调试和替换部件。

## 硬件与功能

- 主控：STM32F407VETx。
- 关节：3 路步进电机与 TMC2209 驱动器。
- 反馈：3 路 AS5600 磁编码器，分别接入 I2C1、I2C2、I2C3。
- 末端：TIM2_CH2 PWM 舵机夹爪。
- 输入：UART1 上位机和 PS2 手柄。
- 安全：3 路限位开关、回零、关节软件限位与故障锁定。
- 运动：逆运动学、直线分段规划、Bresenham 脉冲引擎和位置保持闭环。

## 软件架构

```text
APP → SERVICE → DEVICE → BSP/HAL → Hardware
```

- `app/`：G-code、PS2 遥操作、模式与校准流程。
- `service/`：回零、夹爪服务，以及运动学、规划、闭环等控制算法。
- `device/`：关节、夹爪、限位和操作者输入等设备级接口。
- `bsp/`：STM32 板级 UART 与 LED。
- `os/`：裸机时间与延时适配；后续迁移 FreeRTOS 时在此替换实现。
- `common/`：机械臂配置、数学工具、Home Pose 与错误码。

详细边界与依赖关系见 [架构说明](docs/architecture.md)。

## 构建

工程使用 STM32CubeMX 生成底层初始化代码，并使用 CMake/Ninja 构建。

```powershell
cmake --preset Debug
cmake --build build/Debug --parallel
```

CubeMX 工程文件为 `robotarm_stm32f407.ioc`。重新生成代码前，应确认 CubeMX 不会覆盖 `Core/Inc` 与 `Core/Src` 下的自有分层目录。

## 上位机接入

UART1 使用 115200、8N1、3.3 V TTL。每条命令必须以 `\r\n` 结尾。

```text
G1 X100 Y180 Z160 F2000\r\n
M400\r\n
M5\r\n
```

机械臂在 G-code 模式下接收串口命令；PS2 模式下串口输入会被丢弃。`G0/G1` 的 `ok` 仅表示命令已被接收并加入运动队列。如需确认运动完成，上位机应发送 `M400` 并等待 `ok`。

完整指令、状态回复与异常处理见 [串口协议](docs/serial-protocol.md)。

## 安全与调参

- 初次使用或调整机械结构后，必须验证回零方向、Home Pose 和软件关节限位。
- `RobotArm/Domain/include/common/robot_config.h` 集中保存连杆长度、减速比、速度上限、回零退回角度和限位参数。
- 当前 `GCODE_MAX_FEEDRATE` 与 `MOTOR_MAX_STEP_RATE_HZ` 是保守起点，应结合实际负载和供电进行验证。
- 安全故障锁定后，运动指令将被拒绝；排除原因后使用 `M999` 重新回零与校准。

## 文档

- [软件架构](docs/architecture.md)
- [UART/G-code 串口协议](docs/serial-protocol.md)
