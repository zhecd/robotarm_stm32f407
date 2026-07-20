# robotarm_stm32f407

基于 STM32F407 的三自由度教学机械臂下位机固件。项目面向具备 STM32 与 C 语言基础、正在学习机器人控制软件组织方式的读者，覆盖串口 G-code 控制、PS2 手柄控制、运动规划、回零、编码器反馈和安全保护。

本仓库只负责机械臂下位机：接收上位机或手柄命令，完成运动执行与设备保护；视觉识别、任务决策等功能应放在上位机侧。

## 功能概览

- 三轴步进关节与 TMC2209 驱动；三路 AS5600 单圈磁编码器反馈。
- UART1 G-code 指令控制，支持直线/关节运动、夹爪、回零和队列同步。
- PS2 手柄遥操作，并通过模式状态隔离手柄与串口运动输入。
- 直线分段规划、步进脉冲插补、加减速和末端位置保持。
- 限位开关回零、软件关节限位、编码器偏差检测与故障锁定。
- 命令队列与 `M400` 同步，避免上位机在动作未完成时误判任务状态。
- 主机侧逻辑测试与分层依赖检查，可在不连接开发板的情况下验证核心规则。

## 硬件组成

| 部件 | 当前实现 |
| --- | --- |
| 主控 | STM32F407VETx |
| 关节执行器 | 3 路步进电机 + TMC2209 |
| 关节反馈 | AS5600 × 3，分别使用 I2C1、I2C2、I2C3 |
| 末端夹爪 | TIM2_CH2 PWM 舵机 |
| 人机输入 | UART1 上位机、PS2 手柄 |
| 安全输入 | 3 路限位开关 |

## 软件架构

项目采用“应用编排、业务服务、设备抽象、板级实现、算法与平台支撑”相互配合的结构：

```text
                         UART G-code / PS2 / 限位中断
                                      │
                                      ▼
                         App（解析、模式、流程编排）
                                      │ 使用公开服务接口
                                      ▼
 Service（Command / Planning / Motion / Homing / Gripper / State / Safety）
                         │                         │
                         │                         └── Algorithm（运动学、数学、配置、Home Pose）
                         ▼
       Device（关节、夹爪、输入、限位开关的设备级语义）
                                      │
                                      ▼
 BSP（UART、LED、定时器、I2C、GPIO） + BSP/driver（AS5600、TMC2209、步进、舵机、PS2）
                                      │
                                      ▼
                            STM32 HAL / 硬件

 Platform：时间、延时、临界区等裸机适配能力
```

其中，`App` 不直接调用 Service 的内部控制器；`SafetyService` 负责保存和发布故障状态，`MotionService` 负责立即停止运动。这样可以使故障处理方向保持单向，便于后续扩展与测试。

详细设计请参阅：[软件架构](docs/architecture.md)、[数据流](docs/data_flow.md)、[模块职责](docs/service_ownership.md) 和 [依赖规则](docs/dependency_rules.md)。

## 目录说明

```text
RobotArm/
├─ App/         应用层：G-code 解析与执行、PS2 遥操作、启动与运行流程
├─ Service/     服务层：命令、规划、运动、回零、夹爪、状态和安全服务
├─ Device/      设备层：将底层驱动组合为关节、夹爪、输入和限位设备
├─ BSP/         板级外设与具体器件驱动
├─ Algorithm/   运动学、数学工具、机械臂参数和 Home Pose
└─ Platform/    裸机平台适配：时间、延时和临界区

docs/           架构、接口、命名和串口协议文档
tests/          主机侧逻辑测试
scripts/        静态架构依赖检查脚本
Core/            STM32CubeMX 生成的启动、外设初始化与中断入口代码
```

## 构建固件

工程使用 STM32CubeMX 生成底层初始化代码，并使用 CMake/Ninja 构建。请先安装 ARM GNU Toolchain、CMake、Ninja，以及项目使用的 `cube-cmake` 工具。

```powershell
cmake --preset Debug
cmake --build build/Debug --parallel
```

CubeMX 工程文件为 `robotarm_stm32f407.ioc`。重新生成 CubeMX 代码时，请仅更新 `Core/` 相关生成内容，不要覆盖 `RobotArm/` 下的手写业务代码。

## 串口使用

UART1 参数：115200 baud、8N1、3.3 V TTL。每条命令必须以 CRLF（`\r\n`）结束；串口工具需要勾选“发送新行”或明确发送 `0D 0A`。

```text
G1 X100 Y180 Z160 F2000\r\n   ; 笛卡尔直线运动
G0 Z160\r\n                   ; 笛卡尔直线运动（当前与 G1 执行方式相同）
M3\r\n                        ; 打开夹爪
M5\r\n                        ; 关闭夹爪
M400\r\n                      ; 等待已接收运动全部完成
M999\r\n                      ; 故障后重新回零并恢复
```

- `ok` 表示命令已通过校验并被接收；对运动命令而言，这不等同于动作已经完成。
- `queued` 表示命令已进入等待队列；上位机应根据自身任务策略决定是否继续发送。
- `M400` 返回 `ok` 时，表示此前已接收的运动队列已经执行完成。
- 安全故障锁定后，运动指令会被拒绝；排除机械原因后使用 `M999` 重新回零。

完整的命令格式、返回值和异常处理见 [UART/G-code 串口协议](docs/serial-protocol.md)。面向上位机的函数接口说明见 [接口参考](docs/interface_reference.md)。

## 参数与安全调试

- 机械臂连杆长度、减速比、步进参数、速度限制、软限位和回零退回角度集中在 `RobotArm/Algorithm/include/common/robot_config.h`。
- 标定完成后的零位姿态定义位于 `RobotArm/Algorithm/include/common/robot_home_pose.h`。
- 修改机构尺寸、传动比或限位方向后，必须重新核对运动学参数、回零方向、Home Pose 和软件限位，再以低速验证。
- 编码器为单圈编码器。系统以回零建立绝对参考；运行中检测到实际角度超出安全范围或偏差异常时，会停止运动并进入安全锁定。

## 主机侧回归测试

下列命令使用本机 GCC 构建不依赖 STM32 硬件的逻辑测试：

```powershell
cmake -S tests -B build/host-tests -G Ninja -DCMAKE_C_COMPILER=gcc
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
python scripts/check_architecture_dependencies.py
```

测试覆盖基础运动逻辑、规划服务和安全服务；架构检查用于防止 App 直接依赖内部控制器，或引入 Safety 到 Motion 的反向依赖。

## 相关文档

- [软件架构](docs/architecture.md)
- [数据流与调用关系](docs/data_flow.md)
- [模块职责与接口归属](docs/service_ownership.md)
- [分层依赖规则](docs/dependency_rules.md)
- [接口参考](docs/interface_reference.md)
- [命名规范](docs/naming_conventions.md)
- [UART/G-code 串口协议](docs/serial-protocol.md)
