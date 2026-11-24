# 下位机分步复现与 AI 编程蓝图

> STM32F407 RobotArm · 2026-11-24 · S00–S15

本课程按“总—分—总”推进。S00 先给学生一份可构建、能观察、禁止执行器动作的工程；S01–S14 每步复现 1–2 个积木并接通当步的真实数据流；S15 将各条链路纳入全量回归、ELF 审计和台架验收。本文列出的 `TEACH_*` 是拟用于课程的替身登记名，仓库已有的 Host 替身位于 `tests/support/` 和 `tests/hal_stubs/`。正式教学参数随本版固件冻结。三轴并行回零退回为 M1/M2/M3 **101.7° / 29° / 4°**，对应 **4068 / 1160 / 160 微步**。教师分步种子及检查点需在独立教学副本中发布。

## 1. 正式参数与基线体检

### 1.1 正式回零和角度参数

2026-11-24 定稿按 `ProductionDebug` 与 `ProductionRelease` 选择正式并行回零。回零退回角表示触发限位后退回的相对角度。归位姿态和软限位使用归位后的关节坐标，编码器反馈和闭环误差另以电机轴角度表示。

| 参数 | 正式构建默认值 | 源码依据 |
| --- | --- | --- |
| 回零退回角与微步 | M1/M2/M3 为 **101.7° / 29° / 4°**，对应 **4068 / 1160 / 160 微步**。每电机转 3200 微步，减速比 4.5，即 40 微步/关节度 | `robot_config.h:101–106,148,370–402`，`svc_homing_sequence.c:22–30` |
| 底座回转与关节软限位 | 底座 M1 −87° 至 97°，大臂 M2 −25° 至 93°，小臂 M3 −100° 至 2°。反馈软限检查配置另允许 1° 容差 | `robot_config.h:165–212`，`robot_home_pose.c` |
| 归位姿态与反馈换算 | 归位后三轴参考均为 0°。按当前连杆与工具偏置计算，参考桌面坐标为 (0, 192, 245) mm。电机角按 4.5 减速比及 +/−/− 符号换算，再叠加各轴归位角 | `robot_config.h:127–145,190–206`，`robot_home_pose.c` |
| 闭环角度阈值 | 死区 3° 电机角，恢复窗口 ≤10° 关节角，单轮 ≤1° 关节角，恢复总限 20 s，连续 3 轮改善不足停止 | `robot_config.h:59–79,280–285`，`ctrl_closed_loop.c` |
| 夹爪目标角 | 开爪 100°，闭爪 72°，目标持有 600 ms | `bsp_driver_config.h:9–22` |

正式回零由 SysTick 1 ms 调度，三轴全部稳定触发限位才并行退回。`ROBOTARM_PARALLEL_HOMING=1` 且 `ROBOTARM_STARTUP_TRIAL_MODE=0` 选用上述角度。锁定/单次试验预设和历史发布版各有参数。作为对照，原 GitHub 提交 [9471b53](https://github.com/zhecd/robotarm_stm32f407/commit/9471b53e2f2c2f3d9ee547fb3a0406ae6d63605a) 的旧主循环配置为 100°/31°/5°。

### 1.2 定稿源码复测基线

以下测试与 ELF 证据来自定稿所用的本地源码。Git 状态数字记录的是首次基线检查时的工作树。初始 `git -c core.excludesFile= status --porcelain=v1 --untracked-files=normal` 有 96 条已跟踪修改和 150 条未跟踪记录（目录按 Git 默认折叠），暂存区无记录。已修改项涵盖 `CMakeLists.txt`、`CMakePresets.json`、`robotarm_stm32f407.ioc`、`Core/Src/{main,gpio,usart,stm32f4xx_it}.c`、`robot_config.h` 和 `tests/CMakeLists.txt` 等。新成行器、并行回零实现、审计脚本及多项测试仍是未跟踪内容。课程冻结包须明确纳入自研新文件，并以锁定清单复核解包可构建性。

| 检查 | 2026-11-24 结果 | 对教学发布的影响 |
| --- | --- | --- |
| Host GCC 8.1.0，`HostTests` 与 `HostTestsRelease` | 各重新构建成功，各 **555/559** 通过。4 项 CTest 均报告超过 15 s，详见下表 | S00–S14 各卡现行筛选共执行 90 项次，全部通过，含跨卡重复项。S09 另列的 4 项正常交接回归须在教师修复基线后复跑 |
| Arm GNU 15.2.1 | `TeachingLimitOnly`、`TeachingStartupAudit`、`ProductionLocked`、`ProductionDebug`、`ProductionRelease` 干净构建 | HAL Flash 有未使用形参警告，链接器提示 ELF 有 RWX LOAD segment，构建返回成功。将警告状况与审计状态分别记录 |
| 架构检查 | `python -B scripts/check_architecture_dependencies.py` 通过 | 可作为分步及收尾门禁 |
| 当前 GPIO 审计 | `scripts/audit_firmware.py` 对 `ProductionDebug`、`ProductionRelease` 均在第 48 行失败。对 `ProductionLocked` 按 `--compiled 0 --home-period-ms 1` 通过 | 正式生产镜像尚未达到发布门槛 |
| 锁定试验专用审计 | `tools/startup_trial/audit_binary.py <ELF> --bin-dir <Arm GNU bin> --locked` 对 `ProductionLocked` 通过 | 验证异常向量及输出保护等 ELF 证据，物理脉宽、总线时序和实机动作另行验收 |

本轮构建及报告写在临时目录。没有连接板卡、发送真实串口命令、烧录、复位或执行动作。

### 1.3 验收差异与教师修复顺序

| 发现与位置 | 当前口径 | 统一建议 |
| --- | --- | --- |
| 正式并行回零配置与现有验收断言存在差异。`RobotArm/Algorithm/include/common/robot_config.h:370–402`、`RobotArm/Service/homing/svc_homing_sequence.c:22–30` | `ProductionDebug/Release` 编译参数锁定为 `101.7°/29°/4°`，此前两份 ELF 均读出 `4068/1160/160`。本地测试预期 `4040/1160/160`，本地审计预期 `4000/1160/160` | 教学和正式固件以 §1.1 为准。教师在 S15 签收前须把 Host 与 ELF 验收预期对齐到正式的 4068/1160/160，再复跑并保留错值负对照 |
| 4 个 Host 集成用例红灯。`tests/CMakeLists.txt:827–835`、`tests/parallel_runtime_tests.c:68–93` | `parallel_runtime_normal`、`parallel_runtime_pressed`、`parallel_runtime_recovery`、`parallel_runtime_rate1000` 都进入带 4040 断言的分支，实测 CTest 报 Timeout。另用 GDB 在 normal 的第 79 行读取现场值，得到 4068/1160/160。`parallel_runtime_port/stuck` 走故障分支并通过 | 先统一第 79 行的黄金值，再单独复跑四项和全套。保留超时及断言输出，勿靠延长超时、排除用例或放宽安全检查制造绿灯 |
| 本地锁定/试验预设有独立配置。`robot_config.h:370–402`、`tools/startup_trial/guard.c:247–257,310–315` | 当时本地 `ROBOTARM_STARTUP_TRIAL_MODE=1` 使用 `100°/31°/10° = 4000/1240/400`。纯算法 `scale` 用例是显式测试向量 | 该数值只用于解释对应试验镜像和验收记录。课堂参数表使用 §1.1，教师在具体试验前核对所选镜像和其参数 |
| 审计默认周期和编排。`scripts/audit_firmware.py:19,41–50,113`、`scripts/complete_software.py:103–107`、`scripts/validate_teaching.py:2–4` | 当前 ELF 寻位与退回均为 1 ms。审计 CLI 默认 2 ms，历史一键编排调用未传 1 ms。`validate_teaching.py` 转入同一个全量打包入口 | 当前逐镜像审计显式传 `--home-period-ms 1`。教师发布时同步编排参数；如找不到预期的 `s_config` 设计为审计失败或明确使用替代读取方案，避免漏检。Release 缩减的反汇编检查需要单列记录 |
| CubeMX 输入与守护代码有漂移。`robotarm_stm32f407.ioc:72–85,150–164,244,313–322`、`Core/Src/{gpio,usart,tim}.c` | USART1 RX DMA、实际优先级、GPIO 状态模式和启动钩子位于生成树，部分在 `USER CODE` 之外。实值见第 3、4 节 | 对一次重生成留存 `.ioc` 与 `Core/` 差异，教师逐处确认生成参数和需保留的扩展。得出可重生成的教学源后再冻结 |

### 1.4 参数核对表

| 参数组 | 当前默认值与来源 |
| --- | --- |
| 单位 | `STEPS_PER_REV=3200`（200 整步 ×16），`GEAR_RATIO=90/20=4.5`（电机角/关节角），`UNITS_PER_DEGREE=40` 微步/关节度。见 `robot_config.h:101–121,148` |
| 回零 | 正式退回角与微步见 §1.1。S09 寻位方向位 M1/M2/M3 为 `0/0/1`，两阶段 1 ms，整轮 220000 ms，寻位单阶段超时配置为 0。见本地 `robot_config.h:333–402` 和 `svc_homing_sequence.c:22–30` |
| 运动帧和队列 | `TICKS_PER_MS=50`，步频上限 `MOTOR_MAX_STEP_RATE_HZ=10000` pulse/s；规划持续不能入队超时 `PLANNER_QUEUE_TIMEOUT_MS=2000U`。见 `robot_config.h:218–254,417–418` |
| 闭环 | 正式角度阈值见 §1.1。S10 另有反馈读耗时达到 100 ms 时拒用该次读数的门限。见本地 `robot_config.h:24–91,257–322` |
| 启动与输入 | 编码器准备采样间隔 100 ms、总限 10000 ms；PS2 轮询 10 ms、首次接管期限 750 ms、每次遥操作请求最大增量 0.9 mm。见 `robot_config.h:17–21,436–449` |
| 参数的物理归属 | `RobotArm/BSP/driver/include/bsp_driver_config.h:9–39` 提供夹爪 500–2500 µs、开 100°、闭 72°、保持 600 ms，TMC 默认 16 微步、IRUN 28、IHOLD 15（寄存器值）。`RobotArm/BSP/include/bsp/bsp_uart1_config.h:9–19` 提供 RX 256 B、TX 1024 B、行超时 500 ms |

## 2. 全工程文件分类

主表逐一覆盖当前构建的 `Core/` 与 `RobotArm/` 共 125 个 C/H（Core 10 C、7 H；RobotArm 50 C、58 H）。附录逐项列出 `tests/`、`tools/` 与 `Drivers/` 的当前 C/H。构建输出、交付包和历史快照作为归档材料。根目录启动汇编 `startup_stm32f407xx.s`、链接脚本 `STM32F407VETx_FLASH.ld`、CubeMX 输入、CMake、测试与审计同属教师控制项。

| 文件 | 层级与职责 | 管控类别 |
| --- | --- | --- |
| `Core/Inc/gpio.h` | Core · GPIO 初始化声明 | 教师锁死合同/骨架 |
| `Core/Inc/i2c.h` | Core · 三路 I²C 句柄与初始化声明 | 教师锁死合同/骨架 |
| `Core/Inc/main.h` | Core · 引脚名称和错误入口 | 教师锁死合同/骨架 |
| `Core/Inc/stm32f4xx_hal_conf.h` | Core · HAL 模块与时钟配置 | 教师锁死合同/骨架 |
| `Core/Inc/stm32f4xx_it.h` | Core · 异常与外设 IRQ 声明 | 教师锁死合同/骨架 |
| `Core/Inc/tim.h` | Core · TIM2/TIM6 句柄与初始化声明 | 教师锁死合同/骨架 |
| `Core/Inc/usart.h` | Core · USART1/6 与 RX DMA 句柄 | 教师锁死合同/骨架 |
| `Core/Src/gpio.c` | Core · 绑脚、初值、限位模式与 NVIC | 教师锁死合同/骨架，S00/S01/S03/S06/S14 教师核对 |
| `Core/Src/i2c.c` | Core · I2C1/2/3 初始化、复用与上拉 | 教师锁死合同/骨架，S05 教师核对 |
| `Core/Src/main.c` | Core · 启动顺序、时钟、应用挂接 | 教师锁死合同/骨架，所有 CubeMX 步教师核对 |
| `Core/Src/stm32f4xx_hal_msp.c` | Core · HAL 全局 MSP 初始化 | 教师锁死合同/骨架 |
| `Core/Src/stm32f4xx_it.c` | Core · SysTick、TIM、EXTI、DMA/USART IRQ | 教师锁死合同/骨架，所有 CubeMX 步教师核对 |
| `Core/Src/syscalls.c` | Core · newlib syscall 与标准输出钩子 | 教师锁死合同/骨架 |
| `Core/Src/sysmem.c` | Core · newlib _sbrk 与堆栈边界 | 教师锁死合同/骨架 |
| `Core/Src/system_stm32f4xx.c` | Core · CMSIS SystemInit/SystemCoreClock | 教师锁死合同/骨架 |
| `Core/Src/tim.c` | Core · TIM2 PWM/TIM6 时基、PA1 复用 | 教师锁死合同/骨架，S08/S13 教师核对 |
| `Core/Src/usart.c` | Core · USART1/6 MSP、DMA、NVIC | 教师锁死合同/骨架，S01/S04 教师核对 |
| `RobotArm/Algorithm/common/home_sequence.c` | Algorithm · 纯回零状态机、接触屏障与并行退回 | 积木文件，编写并冻结 S09 |
| `RobotArm/Algorithm/common/limit_state_filter.c` | Algorithm · 低/高稳定计时与发步阻塞 | 积木文件，编写并冻结 S06 |
| `RobotArm/Algorithm/common/robot_home_pose.c` | Algorithm · 归位姿态、方向换算、关节软限 | 教师锁死合同/骨架，S09 使用 |
| `RobotArm/Algorithm/common/robot_math.c` | Algorithm · 微步数绝对值最大值 | 教师锁死合同/骨架，S08/S11 使用 |
| `RobotArm/Algorithm/include/algorithm/kinematics.h` | Algorithm · 机构尺寸、逆解及电机单位接口 | 教师锁死合同/骨架 |
| `RobotArm/Algorithm/include/common/error_code.h` | Algorithm · 公共错误码 | 教师锁死合同/骨架 |
| `RobotArm/Algorithm/include/common/gripper_status.h` | Algorithm · 夹爪状态枚举与数据 | 教师锁死合同/骨架 |
| `RobotArm/Algorithm/include/common/home_sequence.h` | Algorithm · 回零数据与 API | 教师锁死合同/骨架 |
| `RobotArm/Algorithm/include/common/limit_state_filter.h` | Algorithm · 限位滤波状态与 API | 教师锁死合同/骨架 |
| `RobotArm/Algorithm/include/common/robot_config.h` | Algorithm · 几何、执行、回零、规划与闭环参数 | 教师锁死合同/骨架 |
| `RobotArm/Algorithm/include/common/robot_home_pose.h` | Algorithm · 归位与软限坐标 API | 教师锁死合同/骨架 |
| `RobotArm/Algorithm/include/common/robot_math.h` | Algorithm · 微步数工具 API | 教师锁死合同/骨架 |
| `RobotArm/Algorithm/kinematics/kinematics.c` | Algorithm · 逆运动学、角度与电机单位换算 | 积木文件，编写并冻结 S11 |
| `RobotArm/App/adapters/app_hardware_adapter.c` | App Adapter · 传输、反馈、指示灯与执行器接口 | 枢纽文件，改动步骤 S00、S01、S03–S06、S08、S09、S13、S14 |
| `RobotArm/App/adapters/app_hardware_adapter.h` | App Adapter · 板级端口及数据约定 | 教师锁死合同/骨架 |
| `RobotArm/App/adapters/app_isr_adapter.c` | App Adapter · GPIO/TIM/SysTick 分派、限位门禁 | 枢纽文件，改动步骤 S00、S06、S08、S09、S14 |
| `RobotArm/App/adapters/app_isr_adapter.h` | App Adapter · Core 事件入口 | 教师锁死合同/骨架 |
| `RobotArm/App/app_calibration.c` | App · 清理论计数、逐轴置零与三轴准入 | 教师锁死合同/骨架，S09 使用 |
| `RobotArm/App/app_diagnostics.c` | App · 只读诊断入口、限位与受控采样 | 诊断或历史兼容分支，S00–S02 用现成对照，正式主线不启用 |
| `RobotArm/App/app_gcode_exec.c` | App · 向 CommandService 转发 G-code | 教师锁死合同/骨架，S12 使用 |
| `RobotArm/App/app_gcode_parser.c` | App · G-code 词法和字段校验 | 积木文件，编写并冻结 S12 |
| `RobotArm/App/app_runtime.c` | App · 启动/回零/恢复/READY、应用调度 | 枢纽文件，改动步骤 S00、S04、S05、S07–S14 |
| `RobotArm/App/app_serial_protocol.c` | App · 许可、状态、命令回执及异步等待 | 枢纽文件，改动步骤 S02、S10、S12–S14 |
| `RobotArm/App/app_teleop.c` | App · 模式、中立接管、增量帧、夹爪与失联处理 | 教师锁死合同/骨架，S14 启用教师预置实现 |
| `RobotArm/App/app_waveform.c` | App · 隔离波形固件及会话 | 诊断或历史兼容分支，选读 |
| `RobotArm/App/include/app/app_calibration.h` | App · 标定接口 | 教师锁死合同/骨架 |
| `RobotArm/App/include/app/app_diagnostics.h` | App · 诊断入口 | 诊断或历史兼容分支，锁定，随对照读取 |
| `RobotArm/App/include/app/app_gcode_exec.h` | App · 执行及异步结果接口 | 教师锁死合同/骨架 |
| `RobotArm/App/include/app/app_gcode_parser.h` | App · 解析函数 | 教师锁死合同/骨架 |
| `RobotArm/App/include/app/app_runtime_events.h` | App · 模式等运行事件接口 | 教师锁死合同/骨架 |
| `RobotArm/App/include/app/app_serial_protocol.h` | App · 协议状态与任务入口 | 教师锁死合同/骨架 |
| `RobotArm/App/include/app/app_teleop.h` | App · 遥操作任务及模式 | 教师锁死合同/骨架 |
| `RobotArm/App/include/app/app_waveform.h` | App · 波形应用入口 | 诊断或历史兼容分支，锁定，选读 |
| `RobotArm/App/include/app_init.h` | App · App_Init/App_RunOnce 声明 | 教师锁死合同/骨架 |
| `RobotArm/BSP/driver/include/bsp_driver_config.h` | BSP Driver · 舵机及 TMC 参数 | 教师锁死合同/骨架 |
| `RobotArm/BSP/driver/include/driver/drv_as5600.h` | BSP Driver · 编码器绑定、读角、置零接口 | 教师锁死合同/骨架 |
| `RobotArm/BSP/driver/include/driver/drv_ps2.h` | BSP Driver · PS2 数据、配置及诊断接口 | 教师锁死合同/骨架 |
| `RobotArm/BSP/driver/include/driver/drv_servo.h` | BSP Driver · PWM 与目标状态契约 | 教师锁死合同/骨架 |
| `RobotArm/BSP/driver/include/driver/drv_stepper.h` | BSP Driver · STEP/DIR/EN 与轴绑定契约 | 教师锁死合同/骨架 |
| `RobotArm/BSP/driver/include/driver/drv_tmc2209.h` | BSP Driver · TMC 寄存器读写与报告契约 | 教师锁死合同/骨架 |
| `RobotArm/BSP/driver/include/driver/ps2_bus.h` | BSP Driver · PS2 字节交换接口 | 教师锁死合同/骨架 |
| `RobotArm/BSP/driver/src/drv_as5600.c` | BSP Driver · 采样、跨圈角与反馈零偏 | 积木文件，编写并冻结 S05 |
| `RobotArm/BSP/driver/src/drv_ps2.c` | BSP Driver · 手柄配置、帧校验及诊断 | 积木文件，编写并冻结 S14 |
| `RobotArm/BSP/driver/src/drv_servo.c` | BSP Driver · PWM 绑定、目标更新与到期停止 | 积木文件，编写并冻结 S13 |
| `RobotArm/BSP/driver/src/drv_stepper.c` | BSP Driver · 有效低 EN、DIR 与 STEP | 积木文件，编写并冻结 S03 |
| `RobotArm/BSP/driver/src/drv_tmc2209.c` | BSP Driver · UART 报文、CRC、配置回读与静音 | 积木文件，编写并冻结 S04 |
| `RobotArm/BSP/driver/src/ps2_bus.c` | BSP Driver · GPIO 位时序 | 积木文件，编写并冻结 S14 |
| `RobotArm/BSP/include/bsp/bsp_i2c_recovery.h` | BSP · I²C 解锁接口 | 教师锁死合同/骨架 |
| `RobotArm/BSP/include/bsp/bsp_led.h` | BSP · LED 端口 | 教师锁死合同/骨架 |
| `RobotArm/BSP/include/bsp/bsp_limit_trace.h` | BSP · IRQ RAM 证据接口 | 诊断或历史兼容分支，锁定，选读 |
| `RobotArm/BSP/include/bsp/bsp_robot_board.h` | BSP · 板级资源、限位与停机端口 | 教师锁死合同/骨架 |
| `RobotArm/BSP/include/bsp/bsp_uart1.h` | BSP · 串口读行、回包及异常接口 | 教师锁死合同/骨架 |
| `RobotArm/BSP/include/bsp/bsp_uart1_config.h` | BSP · RX/TX 与超时配置 | 教师锁死合同/骨架 |
| `RobotArm/BSP/include/bsp/uart_line_reader.h` | BSP · 成行容量、状态与 API | 教师锁死合同/骨架 |
| `RobotArm/BSP/src/bsp_i2c_recovery.c` | BSP · 有界解锁、STOP 与总线复原 | 积木文件，编写并冻结 S05 |
| `RobotArm/BSP/src/bsp_led.c` | BSP · 四路 LED 控制 | 积木文件，编写并冻结 S01 |
| `RobotArm/BSP/src/bsp_limit_irq.c` | BSP · IRQ 调度与 GPIO 释放确认 | 积木文件，编写并冻结 S06，生产保留 GPIO 分支 |
| `RobotArm/BSP/src/bsp_limit_trace.c` | BSP · IRQ 向量寄存器快照 | 诊断或历史兼容分支，选读 |
| `RobotArm/BSP/src/bsp_robot_board.c` | BSP · TIM6、USART6、限位、TIM2、执行器停机 | 枢纽文件，改动步骤 S00、S03、S04、S06、S08、S13 |
| `RobotArm/BSP/src/bsp_uart1.c` | BSP · DMA RX、IT TX、收发回调与故障 | 积木文件，编写并冻结 S01 |
| `RobotArm/BSP/src/uart_line_reader.c` | BSP · 逐字节成行、超时及坏行丢弃 | 积木文件，编写并冻结 S02 |
| `RobotArm/Device/include/device/dev_gripper.h` | Device · 夹爪控制与结果接口 | 教师锁死合同/骨架 |
| `RobotArm/Device/include/device/dev_input.h` | Device · 逻辑输入与模拟模式 | 教师锁死合同/骨架 |
| `RobotArm/Device/include/device/dev_joint.h` | Device · 轴、反馈、软限与驱动契约 | 教师锁死合同/骨架 |
| `RobotArm/Device/include/device/dev_limit_switch.h` | Device · 限位取样与释放接口 | 教师锁死合同/骨架 |
| `RobotArm/Device/src/dev_gripper.c` | Device · 夹爪服务到舵机的桥接 | 教师锁死合同/骨架，S13 使用 |
| `RobotArm/Device/src/dev_input.c` | Device · PS2 帧转换为逻辑输入 | 教师锁死合同/骨架，S14 使用 |
| `RobotArm/Device/src/dev_joint.c` | Device · 关节到 Stepper/AS5600 的映射、零位与软限 | 积木文件，编写并冻结 S03 |
| `RobotArm/Device/src/dev_limit_switch.c` | Device · 取样与 mask/reset/rearm 板级桥接 | 教师锁死合同/骨架，S06 使用 |
| `RobotArm/Platform/compat/os/os_adapter.c` | Platform · Os API 的时基与延时转发 | 诊断或历史兼容分支，历史兼容，主线保留标定调用 |
| `RobotArm/Platform/compat/os/os_adapter.h` | Platform · Os 兼容接口 | 诊断或历史兼容分支，锁定，主线保留 |
| `RobotArm/Platform/platform_critical.c` | Platform · PRIMASK 保存与恢复 | 教师锁死合同/骨架 |
| `RobotArm/Platform/platform_critical.h` | Platform · 临界区接口 | 教师锁死合同/骨架 |
| `RobotArm/Platform/platform_delay.c` | Platform · 毫秒与微秒延时 | 教师锁死合同/骨架 |
| `RobotArm/Platform/platform_delay.h` | Platform · 延时接口 | 教师锁死合同/骨架 |
| `RobotArm/Platform/platform_time.c` | Platform · HAL tick、DWT 与回绕处理 | 教师锁死合同/骨架 |
| `RobotArm/Platform/platform_time.h` | Platform · 毫秒/微秒与 SysTick 契约 | 教师锁死合同/骨架 |
| `RobotArm/Service/command/command_service.c` | Service · 指令仲裁、坐标、计划与结果 | 积木文件，编写并冻结 S12 |
| `RobotArm/Service/diagnostics/limit_observation_service.c` | Service · 诊断限位观测与计数 | 诊断或历史兼容分支，选读 |
| `RobotArm/Service/gripper/svc_gripper.c` | Service · 运行期许可、夹爪状态与空闲停机 | 积木文件，编写并冻结 S13 |
| `RobotArm/Service/homing/svc_homing.c` | Service · 正式序列门面与旧主循环分支 | 诊断或历史兼容分支，主线保留门面，旧分支选读 |
| `RobotArm/Service/homing/svc_homing_sequence.c` | Service · 1 ms 回零、STEP 与交接 | 积木文件，编写并冻结 S09 |
| `RobotArm/Service/include/closed_loop_status.h` | Service · 闭环状态和诊断值 | 教师锁死合同/骨架 |
| `RobotArm/Service/include/command_service.h` | Service · 指令服务及结果接口 | 教师锁死合同/骨架 |
| `RobotArm/Service/include/command_types.h` | Service · 指令和字段数据类型 | 教师锁死合同/骨架 |
| `RobotArm/Service/include/limit_irq_guard.h` | Service · IRQ 预算与故障位图接口 | 教师锁死合同/骨架 |
| `RobotArm/Service/include/limit_observation_service.h` | Service · 诊断限位接口 | 诊断或历史兼容分支，锁定，选读 |
| `RobotArm/Service/include/motion_service.h` | Service · 运动、闭环和故障接口 | 教师锁死合同/骨架 |
| `RobotArm/Service/include/motion_types.h` | Service · 固定帧、位图和故障枚举 | 教师锁死合同/骨架 |
| `RobotArm/Service/include/planning_service.h` | Service · 路径及增量规划接口 | 教师锁死合同/骨架 |
| `RobotArm/Service/include/safety_service.h` | Service · 许可、回零和故障契约 | 教师锁死合同/骨架 |
| `RobotArm/Service/include/service/svc_gripper.h` | Service · 夹爪与空闲停机接口 | 教师锁死合同/骨架 |
| `RobotArm/Service/include/service/svc_homing.h` | Service · 回零兼容门面契约 | 诊断或历史兼容分支，锁定，主线保留 |
| `RobotArm/Service/include/service/svc_homing_sequence.h` | Service · 回零序列与交接契约 | 教师锁死合同/骨架 |
| `RobotArm/Service/include/state_service.h` | Service · 样本、有效性与新鲜度接口 | 教师锁死合同/骨架 |
| `RobotArm/Service/include/waveform_service.h` | Service · 隔离波形接口 | 诊断或历史兼容分支，锁定，选读 |
| `RobotArm/Service/motion/internal/ctrl_closed_loop.c` | Service Internal · 误差估计、有界补偿与退出 | 积木文件，编写并冻结 S10 |
| `RobotArm/Service/motion/internal/ctrl_closed_loop.h` | Service Internal · 闭环控制内部 API | 教师锁死合同/骨架 |
| `RobotArm/Service/motion/internal/ctrl_motion_engine.c` | Service Internal · 固定帧队列、ISR 发步与故障停止 | 积木文件，编写并冻结 S08 |
| `RobotArm/Service/motion/internal/ctrl_motion_engine.h` | Service Internal · 执行器固定帧与故障接口 | 教师锁死合同/骨架 |
| `RobotArm/Service/motion/motion_service.c` | Service · 帧门面、故障事件和闭环转接 | 积木文件，编写并冻结 S08 |
| `RobotArm/Service/planning/internal/ctrl_planner.c` | Service Internal · 整路径预检、插补和流式出帧 | 积木文件，编写并冻结 S11 |
| `RobotArm/Service/planning/internal/ctrl_planner.h` | Service Internal · 规划内部 API | 教师锁死合同/骨架 |
| `RobotArm/Service/planning/planning_service.c` | Service · 对规划控制器的公开门面 | 教师锁死合同/骨架，S11 使用 |
| `RobotArm/Service/safety/limit_irq_guard.c` | Service · IRQ 预算和锁存，生产保留查询 | 诊断或历史兼容分支，主线保留门禁查询，旧 EXTI 路径选读 |
| `RobotArm/Service/safety/safety_service.c` | Service · 回零许可、故障锁存和恢复准入 | 积木文件，编写并冻结 S07 |
| `RobotArm/Service/state/state_service.c` | Service · 三轴反馈、故障计数和新鲜度快照 | 积木文件，编写并冻结 S07 |
| `RobotArm/Service/waveform/waveform_service.c` | Service · 隔离台架波形和有界停机 | 诊断或历史兼容分支，选读 |

## 3. S00–S15 复现总表与执行卡

| 阶段 | 步骤 | 积木 | 当步反馈 |
| --- | --- | --- | --- |
| 一 | S00 禁动骨架 | 教师预置 | 双构建、输出锁、时间入口 |
| 一 | S01 串口与 LED | UART BSP、LED BSP | DMA 环回卷与驱动错误可观察 |
| 一 | S02 完整行与身份 | 纯成行器 | M115 单次回包，动作计数为 0 |
| 二 | S03 步进底座 | STEP 驱动、逻辑关节 | 假 GPIO 记录 EN/DIR/STEP |
| 二 | S04 TMC2209 | 芯片通信与校验 | 3 节点配置有回读，失败节点明确 |
| 二 | S05 AS5600 | 编码器、I²C 总线恢复 | 每轴采样、跨圈与读失败 |
| 二 | S06 限位 | 纯滤波、板级限位端口 | 低电平、抖动、稳定释放 |
| 二 | S07 状态与门禁 | State、Safety | 有效反馈、许可与首故障 |
| 三 | S08 固定帧 | 定时执行器、MotionService | 50 kHz 分配步数及拒绝 |
| 三 | S09 并行回零 | 纯序列、1 ms 服务 | 接触屏障、退回、真实置零 |
| 三 | S10 有界闭环 | 闭环控制器 | 死区、收敛与停止原因 |
| 四 | S11 逆解与规划 | Kinematics、Planner | 整段预检后才提交 |
| 四 | S12 G-code | CommandService、Parser | 受理、完成、急停和恢复 |
| 五 | S13 夹爪 | PWM 驱动、夹爪服务 | 启动闭爪、600 ms、运行门禁 |
| 五 | S14 PS2 | 软件总线、手柄驱动 | 中立接管、松杆与失联 |
| 六 | S15 集中验收 | 组合与回归 | 软件、镜像与台架分别签收 |

### 共用执行合同

1. 教师发布同一个冻结起点，冻结 SHA-256 清单并锁定所有 `.h`、`.ioc`、生成树、CMake 双工具链及 Host 测试。学生只写当步列出的 C 文件。教师根据选定参数集中维护配置头并复测相关验收，学生不回改已经冻结的积木。教师预填的薄门面、Platform、测试端口和诊断对照见 §2。
2. 未上线模块在其目标文件内保留可链接的同签名替身，教师登记返回语义、调用点、退场步和负对照。后续替换该文件的符号实现即可，已经冻结的调用方保留。禁止同时链接两个同名强符号。对动作端口默认拒绝，对无输出的 Host 专用确认替身允许注入成功与失败。诊断参考固件和各步临时接线在教师教学副本中维护。
3. 现有 `.ioc` 是全外设参考。首次登场指第一次启用该外设的教学端口，并在 CubeMX 现场核对。教师种子保留构建所需的其他生成符号，未登场的业务端口由锁定骨架隔离。每次确认生成，先保存差异并由教师回读 `git diff -- Core/`，检查 `App_Init`、`App_RunOnce`、GPIO 初值和中断路由，再检查 `.ioc`、启动汇编与 `cmake/stm32cubemx/`。没有 CubeMX 操作的步骤，`git diff -- Core/` 应为空。
4. 每步在前一检查点先运行筛选测试并保留预期失败，按白名单让 AI 实现，重新构建并复跑同一筛选，再运行之前已冻结的门槛。失败须来自本步缺失行为。链接错误、历史断言冲突、零测试命中和测试超时分别记录。S00 是教师基线验收，S15 是发布验收，两步都无学生积木。
5. 每卡的 `H` 栏依次给出真实构建 target 和 CTest 正则，`A` 栏给出真实 ARM 预设。命令从源码根执行。将卡片给出的目标拆成 PowerShell 数组 `$targets`，将正则和预设分别赋给 `$filter`、`$armPreset`。测试列表应命中，结束以 CTest 退出码为准。

```powershell
cmake --preset HostTests
cmake --build --preset HostTests --target @targets
ctest --test-dir build/HostTests -N -R $filter
ctest --test-dir build/HostTests --output-on-failure --no-tests=error -R $filter
cmake --preset $armPreset
cmake --build --preset $armPreset
git diff --check
git diff --name-only
git status --short
```

通过后明确 `git add -- <本步列明的文件>`，用各卡的提交消息。教师的生成补丁单独提交；不使用 `git add .`。保留前测失败、复跑通过、白名单差异、ARM 退出码。课堂 S00–S14 仅做 Host 和镜像验收，真实执行器台架验收归 S15。卡中 `ProductionDebug` 是离线编译参考，上电会自动闭爪并启动并行回零。

### S00 禁动骨架

- **CubeMX**　首次核对 STM32F407VET6，LQFP100，SWD PA13/PA14，HSI 16 MHz 经 PLL M=8、N=168、P=2、Q=4 得 SYSCLK 168 MHz。AHB 168 MHz，APB1 42 MHz（定时器 84 MHz），APB2 84 MHz（定时器 168 MHz）。SysTick 1 ms、优先级 15/0。以 `.ioc:14,27–28,81–83,95–98,226–229,262–292,317–318` 及 `Core/Src/main.c:51–78` 为准。检查 PD2 共用 EN 上电高、三 STEP 低，见 S03。Core 差异重点是时钟、主循环与安全电平。
- **白名单与替身**　学生无实现文件。教师在 `RobotArm/App/app_runtime.c` 固定 `App_Init/App_RunOnce` 的只读骨架，在 `RobotArm/App/adapters/app_hardware_adapter.c` 和 `RobotArm/BSP/src/bsp_robot_board.c` 保留安全停机，在 `RobotArm/App/adapters/app_isr_adapter.c` 保留无动作的事件入口。预放 `TEACH_UART_TRANSPORT`、`TEACH_M115_UNBOUND` 和 `TEACH_ACTUATORS_LOCKED`，本步无退场项。现成 `TeachingLimitOnly` 是参考实现。
- **TDD**　H `platform_time_tests stop_actuators_tests`，`^(platform_time_tests|stop_actuators_tests|architecture_dependencies)$`。A `TeachingLimitOnly`。
- **反馈与提交**　正常给时基推进，Host 单调计时；异常用早到 SysTick、空定时器句柄停机和已处于活动状态的输出，确认安全停机或保持静默。EN/STEP/PWM/TIM6 动作增量为 0 的判据在 S02 集成测试收齐。教师检查点 `S00: lock baseline`。

### S01 UART 与状态灯

- **CubeMX**　USART1 异步，PA9/PA10（TX/RX，AF7），GPIO 无拉、低速（`.ioc:91–94,113–116,313–314`）。`Core/Src/usart.c:34–57,105–141` 给出 115200、8N1、无硬件流控、16 倍采样，RX 为 DMA2 Stream2/Channel 4、循环、外设到内存、字节对齐、内存自增、DMA 高优先级、FIFO 关。USART1/DMA2 Stream2 NVIC 实际 1/0。LED PC0–PC3 推挽、无拉、低速、初值高（`.ioc:140–149,165–174`）。Core 差异检查 DMA 句柄、IRQ、优先级、回调和灯初值。
- **白名单与替身**　`RobotArm/BSP/src/bsp_uart1.c`、`RobotArm/BSP/src/bsp_led.c` 写完冻结。`RobotArm/App/adapters/app_hardware_adapter.c` 接 UART 初始化、收发委托和 LED 端口。`TEACH_UART_TRANSPORT` 退场。教师留 `TEACH_UART_LINE_BASIC`，支持短 ASCII 行、空行与失效后丢到下个行边界，供回卷用例运行。
- **TDD**　H `bsp_uart_tests`，`^bsp_uart_(completed_wrap|deferred_wrap|discard_wrap|deferred_overwrite)$`。A `TeachingLimitOnly`。
- **反馈与提交**　正常跨环能读出 `M119` 行。DMA 回卷通知迟到时等回调且不误报；额外覆盖整环时拒绝残行并报告 RX 溢出。完整成行及全 BSP 组合测试排到 S02。`S01: bind uart and indicators`。

### S02 完整行与 M115 身份

- **CubeMX**　无需打开 CubeMX。复用 S01 UART。Core 差异应为空。
- **白名单与替身**　`RobotArm/BSP/src/uart_line_reader.c` 实现并冻结。`RobotArm/App/app_serial_protocol.c` 仅接通只读 `AppSerial_Service` 中的 M115，保留教师预置查询与动作拒绝。用教师预置查询分发或 `TEACH_GCODE_QUERY_ONLY` 支撑 M115，正式 parser 在 S12。退场 `TEACH_UART_LINE_BASIC` 和 `TEACH_M115_UNBOUND`。
- **TDD**　H `uart_line_reader_tests bsp_uart_tests diagnostic_minimal_runtime_tests`，`^(uart_line_reader_tests|bsp_uart_.*|diagnostic_minimal_runtime_tests)$`。A `TeachingLimitOnly`。
- **反馈与提交**　`M115\r\n` 仅一次身份回包，含 `PROTOCOL:DIAG3`、`MOTOR_ENABLE:LOCKED`、`ENCODER_POLL:0`。超长、半行超时或 NUL 加动作后缀不得产生命令，下一合法行恢复。直接给 `G1 X1`、`M3`、`M999`、`M112` 均按只读模式拒绝，计数为 0。S12 正式协议会另行验证 M112。`S02: validate lines and locked identity`。

### S03 STEP、DIR、EN 与逻辑关节

- **CubeMX**　STEP M1/M2/M3 为 PA6/PA7/PB0，推挽、无拉、极高速、初值低。DIR 为 PD3/PD4/PD5，推挽、无拉、低速、初值低。共用 EN 为 PD2，有效低，初始化高。见 `.ioc:99–121,183–199`、`Core/Src/gpio.c:58–101,138–143`。Core 差异重点为锁存电平先于输出初始化、方向定义与既有启停顺序。
- **白名单与替身**　`RobotArm/BSP/driver/src/drv_stepper.c`、`RobotArm/Device/src/dev_joint.c` 实现并冻结，一次写齐最终轴映射与反馈调用合同。`RobotArm/App/adapters/app_hardware_adapter.c` 接 `Dev_Joint_Init`、`AppHardware_StopJointSteps`；`RobotArm/BSP/src/bsp_robot_board.c` 接最终板级断能。退场 `TEACH_STEPPER_PORT`，保留 AS5600 和 TMC 专用替身。
- **TDD**　H `homing_gpio_default_tests homing_gpio_override_tests`，`^homing_gpio_(default|override)_tests$`。A `ProductionLocked`。
- **反馈与提交**　替身 GPIO 验证有效轴、DIR 极性与 EN 禁用。请求非法轴不输出；方向预置时守住无 STEP 上升沿。此 Host 目标链接旧回零门面，用来验证端口。`S03: freeze step and joint ports`。

### S04 TMC2209 读写与回读

- **CubeMX**　USART6 异步 PC6/PC7（TX/RX，AF8），无拉、极高速（`.ioc:175–178,315–316`）。`Core/Src/usart.c:62–86,146–166` 为 115200、8N1、16 倍采样、无硬件流控。总线节点 0–2，由 `bsp_robot_board.c:59–91` 验证。Core 差异重点是 USART6 配置和 EN 高。
- **白名单与替身**　`RobotArm/BSP/driver/src/drv_tmc2209.c` 冻结。`RobotArm/BSP/src/bsp_robot_board.c` 接 `BSP_RobotBoard_PrepareMotionDrivers` 到已回读的配置/静音接口；`RobotArm/App/adapters/app_hardware_adapter.c` 转发失败节点；`RobotArm/App/app_runtime.c` 在启动检查消费失败结果。退场 `TEACH_TMC_NODE_VERIFY`。
- **TDD**　H `tmc2209_driver_tests tmc2209_bus_model_tests`，`^(tmc2209_(driver|bus_model)_tests|tmc_quiet_(nodes|dropped|crc|readback))$`。A `ProductionLocked`。
- **反馈与提交**　返回正确寄存器、CRC 和 IFCNT 增量才记为配置成功。回读错、CRC 错、丢写或节点缺席均保持禁动，报告失败节点。`S04: verify tmc register writes`。

### S05 AS5600 与 I²C 恢复

- **CubeMX**　I2C1 PB8/PB9、I2C2 PB10/PB11、I2C3 PA8/PC9，依次为 SCL/SDA。100 kHz、7 位地址、AF4 开漏、上拉、极高速，以 `.ioc:7–12,109–112,122–139,179–182`、`Core/Src/i2c.c:42–105,132–195` 为准。Core 差异检查 I2C2 双引脚生成、DeInit 复位对称性及三总线上拉。
- **白名单与替身**　`RobotArm/BSP/driver/src/drv_as5600.c`、`RobotArm/BSP/src/bsp_i2c_recovery.c` 冻结。`RobotArm/App/adapters/app_hardware_adapter.c` 接 `AppHardware_ReadMotorAngle` 到已冻结的 Joint，`RobotArm/App/app_runtime.c` 接启动采样与有效性结果。退场 `TEACH_AS5600_READ`。
- **TDD**　H `as5600_driver_tests i2c_recovery_tests`，`^(as5600_driver_tests|i2c_recovery_tests)$`。A `ProductionLocked`。
- **反馈与提交**　7 位地址 0x36，HAL 左移后 0x6C，角度寄存器 0x0C，验证五点圆均值与跨圈连续性。故意保持 SDA 低，恢复只能限次；读失败保留错误并拒绝本轮样本，不写入伪零值（`drv_as5600.h:20–28`）。`S05: connect bounded encoder reads`。

### S06 GPIO 限位与纯滤波

- **CubeMX**　PC10/PC11/PC12 为 M1/M2/M3 限位、低有效。`.ioc:74,150–164` 是 EXTI15_10 falling 参考，未显式写上拉。生产 `Core/Src/gpio.c:116–154` 的 `ROBOTARM_LIMIT_STATE_MODE=1` 用普通输入加上拉，清三根线的 IMR、EMR、RTSR、FTSR 及挂起。共用向量仍可存在。教师在首次配置时保全两种入口，纯滤波实现不进入 CubeMX。Core 差异检查生产分支、BOOT 就绪通知先于 NVIC、旧 EXTI 对照仍可识别。
- **白名单与替身**　`RobotArm/Algorithm/common/limit_state_filter.c` 与 `RobotArm/BSP/src/bsp_limit_irq.c` 冻结。`RobotArm/BSP/src/bsp_robot_board.c` 接三轴低有效读口；`RobotArm/App/adapters/app_hardware_adapter.c` 接 `AppHardware_IsLimitTriggered` 与 LED2–4 观测；`RobotArm/App/adapters/app_isr_adapter.c` 接 `App_ISR_OnLimitsConfigured` 及采样入口。退场 `TEACH_LIMIT_RAW`、`TEACH_LIMIT_UNQUALIFIED`。
- **TDD**　H `limit_state_filter_tests limit_state_port_tests`，`^(gpio_filter_(held|glitch|bounce|unstable|gap|wrap)|gpio_state_port)$`。A `ProductionLocked`。
- **反馈与提交**　稳定低产生触发判定，稳定高并复读后才确认释放。毛刺、断采样和抖动需阻塞风险步或维持待确认，GPIO rearm 不打开 EXTI。`S06: confirm sampled limits`。

### S07 State 与 Safety

- **CubeMX**　无需打开 CubeMX。Core 差异应为空。
- **白名单与替身**　`RobotArm/Service/state/state_service.c`、`RobotArm/Service/safety/safety_service.c` 冻结。`RobotArm/App/app_runtime.c` 接反馈刷新、故障消费及参考/许可。退场 `TEACH_FEEDBACK_SNAPSHOT`、`TEACH_ACTION_DENIED` 在服务层的临时代码。硬件禁动仍归教师输出边界。
- **TDD**　H `safety_service_tests state_feedback_tests`，`^(safety_service_tests|state_feedback_(axes|invalid|wrap|slow)_tests)$`。A `ProductionLocked`。
- **反馈与提交**　正常三路反馈给有效性和年龄，完成回零且无故障才允许动作。迟到/无效样本保持不可用，首故障锁存撤销许可和参考，清故障后也须重新取得合法参考。`S07: enforce feedback and safety state`。


### S08 固定帧与 TIM6

- **CubeMX**　启用 TIM6 基本定时，无 STEP 通道复用。APB1 定时器 84 MHz、PSC=83、ARR=19，向上计数、ARPE 开，50 kHz（20 µs/tick）。`.ioc:84,309–312,318,321–322` 与 `Core/Src/tim.c:86–107,137–143` 若优先级不同，保留 Core 实际 2/0。Core 差异核对 TIM6_DAC 向量转发和 HAL 周期回调。
- **白名单与替身**　`RobotArm/Service/motion/internal/ctrl_motion_engine.c`、`RobotArm/Service/motion/motion_service.c` 冻结。`RobotArm/App/adapters/app_isr_adapter.c` 接 TIM6 到发步前限位门、`MotionService_OnStepTickFromISR`；`RobotArm/BSP/src/bsp_robot_board.c` 和 `RobotArm/App/adapters/app_hardware_adapter.c` 接计时器端口；`RobotArm/App/app_runtime.c` 保持真实计时器启动门禁。退场 `TEACH_FRAME_EXECUTOR`。闭环维持 `TEACH_CL_NO_CORRECTION`。
- **TDD**　H `motion_engine_tests isr_routing_0_tests`，`^(motion_engine_tests|isr_routing_0_tests)$`。A `ProductionLocked`。路由测试中的旧分支作为向量和门禁对照，S09 再覆盖生产 GPIO 路径。
- **反馈与提交**　Host 受理定长正反向帧，逐轴步数与给定值一致；坏帧、队列满、故障撤销以及非 TIM6 计时入口均不得产生错误脉冲。`S08: execute fixed motion frames`。

### S09 并行回零与归位标定

课堂回零退回按正式 M1/M2/M3 **101.7° / 29° / 4°** 和 **4068 / 1160 / 160 微步** 记录。使用正式镜像做台架教学前，教师先按 §1.3 核对回归与 ELF 审计。

- **CubeMX**　无需打开 CubeMX。复用 S00 SysTick、S03 GPIO、S05 I²C 和 S06 限位。Core 差异为空，复读 SysTick 是否仍转发两类时基事件。
- **白名单与替身**　`RobotArm/Algorithm/common/home_sequence.c`、`RobotArm/Service/homing/svc_homing_sequence.c` 冻结。`RobotArm/App/app_runtime.c` 接启动就绪、回零、100 ms 无步保持及教师标定入口；`RobotArm/App/adapters/app_isr_adapter.c` 接 1 ms 序列，`RobotArm/App/adapters/app_hardware_adapter.c` 在 Host 夹爪端口提供可失败的无 PWM 确认。退场 `TEACH_HOME_NO_PULSES`、`TEACH_ZERO_UNAVAILABLE`，零位改为经真实 Device 的窄桥。保留 `TEACH_GRP_BOOT_ACK`、`TEACH_CL_NO_CORRECTION`、`TEACH_GCODE_BOOT_COORD`、`TEACH_PS2_INACTIVE`。依赖交接见 §4。
- **TDD**　H `parallel_homing_tests home1000_tests parallel_runtime_tests`，`^(parallel_homing_(barrier|stuck|deadline)|home1000_(parallel|same_ms|gap|barrier|deadline)|parallel_runtime_(stuck|port))$`。教师按 §1.1 对齐 Host 预期和 ELF 审计后，补 `^parallel_runtime_(normal|pressed|recovery|rate1000)$`。A `ProductionLocked`，正式参数另以 `ProductionDebug` 离线核对。
- **反馈与提交**　各轴并行寻位，三个稳定接触全部确认后才并行退回。按 1 ms 最快一步调度，断采样不追步。卡住、端口失败或逾期会停止输出并保持参考无效。Host 可检查交接状态与微步计数，教学板保持禁动。`S09: home in parallel and calibrate`。

### S10 有界闭环补偿

- **CubeMX**　无需打开 CubeMX。复用 I²C 与 TIM6，Core 差异应为空。
- **白名单与替身**　`RobotArm/Service/motion/internal/ctrl_closed_loop.c` 冻结，实现包括 S09 已用真实置零桥的同签名语义。`RobotArm/App/app_runtime.c` 接 `MotionService_UpdateClosedLoop` 的调用条件及故障观察，`RobotArm/App/app_serial_protocol.c` 接 M411。退场 `TEACH_CL_NO_CORRECTION`；S08 MotionService 接口不需要改动。
- **TDD**　H `closed_loop_tests closed_loop_window_tests closed_loop_feedback_timing_tests`，`^(closed_loop_tests|closed_loop_feedback_timing_tests|recovery_window_(deadband|converge|outside_0_pos|stalled|timeout_busy|encoder|soft_limit))$`。A `ProductionDebug`。
- **反馈与提交**　死区内不给补偿，窗口内按轮收敛。迟到读数不补偿，超过窗口、20 s 期限或连续 3 轮改善不足时停止并保留故障原因。正式角度阈值见 §1.1。M411 可核对序号、误差和退出状态。`S10: bound closed-loop recovery`。

### S11 逆运动学、预检与规划器

- **CubeMX**　无需打开 CubeMX。Core 差异应为空。
- **白名单与替身**　`RobotArm/Algorithm/kinematics/kinematics.c`、`RobotArm/Service/planning/internal/ctrl_planner.c` 冻结。经教师预置 `planning_service.c` 的门面，在 Host 立即串接 `Kinematics_Solve`、预检、插补及成帧，替换 `TEACH_IK_UNREACHABLE`、`TEACH_PLANNER_REJECT`。在 `RobotArm/App/app_runtime.c` 的 Host 交接中接 `PlanningService_Init` 校验归位坐标，替换 `TEACH_GCODE_BOOT_COORD` 的临时坐标接受判断。S12 将该位置完整交给 `App_GCodeExec_Init`，串口动作路由留在 S12。
- **TDD**　H `planning_service_tests`，`^planning_service_tests$`。A `ProductionDebug`。联合 parser 的 `robotarm_logic_tests` 排到 S12。
- **反馈与提交**　可达的短直线预检全段，然后才更新计划坐标并流式出帧。端点不可达、中间点越界、非法 feed 或持续无法入队须拒绝或锁存相应故障，失败预检不提交计划。`S11: preflight paths before streaming`。

### S12 命令服务与完整 G-code

- **CubeMX**　无需打开 CubeMX。Core 差异应为空。
- **白名单与替身**　`RobotArm/Service/command/command_service.c`、`RobotArm/App/app_gcode_parser.c` 冻结。`RobotArm/App/app_serial_protocol.c` 接查询、`G1/M400/M112/M999`；`RobotArm/App/app_runtime.c` 消费急停与恢复事件，经教师预置的 `app_gcode_exec.c` 委托命令服务。退场 `TEACH_GCODE_QUERY_ONLY`、S11 的临时规划初始化接线和 `TEACH_COMMAND_REJECT`。
- **TDD**　H `command_service_tests robotarm_logic_tests serial_reply_flow_tests reference_protocol_tests firmware_integration_tests`，`^(command_service_tests|robotarm_logic_tests|firmware_integration_tests|serial_reply_flow_(queries|burst|ordered|settled)|reference_protocol_(revoked|fault|restored))$`。A `ProductionDebug`。
- **反馈与提交**　合法 G1 返回受理，等待 M400 经规划、帧队列与闭环就绪才完成。无故障 M999 拒绝；M112 撤销旧队列和参考；有故障且符合条件的 M999 显式启动新一轮检查、回零和标定，最终 READY 后才回 `M999OK`。坏坐标、旧参考或限位拒绝按原错误语义回包。`S12: route commands stop and recovery`。

### S13 启动闭爪与运行期夹爪

- **CubeMX**　TIM2 CH2／PA1，AF1、无拉、低速。APB1 定时器 84 MHz、PSC=83、ARR=19999、CCR 初值 500，PWM1 高有效，50 Hz／20 ms（1 µs/计数），见 `.ioc:87–90,302–308,319–320` 与 `Core/Src/tim.c:45–79,155–168`。Core 差异核对通道、复用和初始化不启动 PWM。
- **白名单与替身**　`RobotArm/BSP/driver/src/drv_servo.c`、`RobotArm/Service/gripper/svc_gripper.c` 冻结。`RobotArm/BSP/src/bsp_robot_board.c`、`RobotArm/App/adapters/app_hardware_adapter.c` 将启动端口通过教师预置 `dev_gripper.c` 接真实驱动。`RobotArm/App/app_runtime.c` 消费启动结果并在 HOMING 调 `Svc_Gripper_IdleStop`，`RobotArm/App/app_serial_protocol.c` 接运行期开合和状态。退场 `TEACH_GRP_BOOT_ACK`、`TEACH_GRP_RUNTIME_DENIED`。
- **TDD**　H `servo_driver_tests gripper_result_chain_tests startup_readiness_tests`，`^(servo_driver_tests|gripper_(result_(init_success|init_failure|permission|stop_failure)|chain_(rapid|rapid_permission))|startup_gripper_(hold_homing|failure|recovery)_tests)$`。A `ProductionDebug`，隔离编译对照 `TeachingGripperPrepare`。
- **反馈与提交**　启动闭爪 72°，运行开爪 100°，接受新目标保持 600 ms 后停 PWM。同一目标不续时，切换目标更新 CCR 并重计时。PWM 启动失败禁止进入本轮回零，运行期开合缺许可须拒绝。M410 仅回请求状态和 `POSITION:UNMEASURED`。`S13: bind gripper startup and permission`。

### S14 PS2 接入与模式切换

- **CubeMX**　PE12 DAT 输入上拉；PE13 CMD、PE14 CS、PE15 CLK 为推挽、无拉、高速，Core 初始化低，`Ps2Bus_Init` 绑定后置高。SW7 `KEY_MODE` 为 PE2，上拉、下降沿、EXTI2 0/0。见 `.ioc:75,200–225`、`Core/Src/gpio.c:68,76–80,103–114,145–147`、`ps2_bus.c:26–44`。Core 差异核对 EXTI2 路由与 UART RX DMA，PS2 位操作屏蔽 IRQ 时依赖独立 DMA 写入。
- **白名单与替身**　`RobotArm/BSP/driver/src/ps2_bus.c`、`RobotArm/BSP/driver/src/drv_ps2.c` 冻结。`RobotArm/App/adapters/app_hardware_adapter.c` 接教师预置的 `dev_input.c`，`RobotArm/App/adapters/app_isr_adapter.c` 路由 PE2，`RobotArm/App/app_runtime.c` 接教师预置的 `app_teleop.c`、模式指示与切换，`RobotArm/App/app_serial_protocol.c` 接 M412 和模式限制。退场 `TEACH_PS2_INACTIVE`、`TEACH_PS2_INVALID_FRAME`。
- **TDD**　H `ps2_driver_tests teleop_lifecycle_tests teleop_continuity_tests`，`^(ps2_driver_(timing|mapping|header|diag_unknown)|teleop_(neutral|loss|loss_recovery)_tests|teleop_continuity_(continuity|bound|neutral|loss|gcode))$`。A `ProductionDebug`。PS2 Host driver 用例替换软件总线端口，实际位宽与 IRQ 影响留给 S15 测量。
- **反馈与提交**　切档需空闲。10 ms 轮询、750 ms 初次协商宽限，中立帧接管，松杆不新增帧并让已受理的帧排空。双键冲突重置接管，坏帧需按合同处理，失联撤销帧和参考并停机。LED1 表示模式，LED2–4 继续表示各限位。`S14: connect ps2 mode and input loss`。

### S15 干净构建、回归与台架

- **CubeMX**　无需打开 CubeMX。检查从教师冻结点至此的 `.ioc` 和 `Core/` 差异，最终再次核对输出初值、实际 IRQ 路由和强符号。
- **白名单与替身**　学生无新增积木或枢纽改动。归档产物、参数、测试日志、审计与台架表。正式镜像中 `TEACH_*` 运行占位全部退场，锁定/诊断预设保留各自独立行为。
- **TDD**　Host Debug/Release 全量；ARM `ProductionDebug`、`ProductionRelease`、独立锁定对照 `ProductionLocked`。采用空构建目录和本节命令，记录真实退出码、编译宏与 ELF SHA-256。
- **反馈与提交**　正常验证身份、并行回零、空载短移动、等待完成和 PS2/夹爪。负对照覆盖不合法路径、限位、失联、M112 与 M999 后旧参考不能复用。待所有验收签收，提交 `S15: record regression and bench acceptance`。

```powershell
$verify = Join-Path $env:TEMP ('robotarm-s15-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
$failed = @()
foreach ($p in @('HostTests','HostTestsRelease')) {
    $b = Join-Path $verify $p
    cmake --preset $p -B $b
    if ($LASTEXITCODE -ne 0) { throw "configure failed for $p" }
    cmake --build $b
    if ($LASTEXITCODE -ne 0) { throw "build failed for $p" }
    ctest --test-dir $b --output-on-failure
    if ($LASTEXITCODE -ne 0) { $failed += "Host suite $p" }
}
foreach ($p in @('ProductionDebug','ProductionRelease','ProductionLocked')) {
    $b = Join-Path $verify $p
    cmake --preset $p -B $b
    if ($LASTEXITCODE -ne 0) { throw "configure failed for $p" }
    cmake --build $b
    if ($LASTEXITCODE -ne 0) { throw "build failed for $p" }
}
python -B scripts/check_architecture_dependencies.py
if ($LASTEXITCODE -ne 0) { $failed += 'architecture' }
$audit = Join-Path $verify 'audit'
python -B scripts/audit_firmware.py --name ProductionDebug --build-dir (Join-Path $verify 'ProductionDebug') --output $audit --home-period-ms 1
if ($LASTEXITCODE -ne 0) { $failed += 'ProductionDebug audit' }
python -B scripts/audit_firmware.py --name ProductionRelease --build-dir (Join-Path $verify 'ProductionRelease') --output $audit --home-period-ms 1
if ($LASTEXITCODE -ne 0) { $failed += 'ProductionRelease audit' }
python -B scripts/audit_firmware.py --name ProductionLocked --build-dir (Join-Path $verify 'ProductionLocked') --output $audit --compiled 0 --home-period-ms 1
if ($LASTEXITCODE -ne 0) { $failed += 'ProductionLocked audit' }
$armBin = Split-Path -Parent (Get-Command arm-none-eabi-nm).Source
$lockedElf = Join-Path $verify 'ProductionLocked/robotarm_stm32f407.elf'
python -B tools/startup_trial/audit_binary.py $lockedElf --bin-dir $armBin --locked
if ($LASTEXITCODE -ne 0) { $failed += 'startup trial audit' }
Get-ChildItem -LiteralPath $verify -Filter 'robotarm_stm32f407.elf' -Recurse | Get-FileHash -Algorithm SHA256
if ($failed.Count) { throw ('S15 gate failed - ' + ($failed -join ', ')) }
```

台架由教师在软件清障和独立现场授权后操作。先用锁定镜像记录 EN 高、三路 STEP 上升沿为 0、TIM2 CH2 无 PWM、TIM6 未启动，再在空载、M3 有支撑的条件下使用已审计正式镜像。记录 `M115`、`M408`、`M119`、`M126` 与反馈年龄，测三轴接触屏障、退回微步总数及两条发步路径的脉宽，闭爪波形记录保持区间。用可达短距离 G1 与 M400 建立正常证据，再验证越界、停止和一次明确的恢复。现场须分别签收 Host、ELF、物理脉冲及机械结果。



## 4. 三处代码级防返工防线

### 4.1 重生成后检查入口、初值和实际向量

| 文件及行号 | 必保留的实际代码 |
| --- | --- |
| `Core/Src/main.c:16–22,40–47,84–106` | `USER CODE Includes` 引入 App/适配声明，`USER CODE 2` 中 `App_Init()` 必在外设初始化之后，`USER CODE 4` 有 `HAL_TIM_PeriodElapsedCallback` 和 `HAL_GPIO_EXTI_Callback`。当前 `App_RunOnce()` 和试验 `Error_Handler` 钩子处于区外，重生成后须确认主循环仍持续运行一次 App 并保留试验断能 |
| `Core/Src/stm32f4xx_it.c:24–31,64–66,77–159,206–218,231–287` | `SysTick_IRQn 1` 在 `HAL_IncTick()` 后更新 PlatformTime，并按宏转发 App 1 ms 事件。DMA2 Stream2 句柄声明与实际 IRQ、试验异常断能在区外。`EXTI15_10_IRQn 0` 进入记录后保留 BSP dispatch 与退出钩子，注重区外调用。保留 EXTI2、USART1、TIM6 的 HAL 转发 |
| `Core/Src/gpio.c:24–29,58–155` | 先写输出锁存 EN 高、STEP 低，再配置推挽。限位输入的上拉、生产 GPIO 分支和配置完成钩子须在相关 NVIC 启用前完成。GPIO 状态模式保持三条限位 EXTI 关闭，PE2 模式键仍沿自身 EXTI 转发 |
| `Core/Src/usart.c:117–141`、`Core/Src/tim.c:140–143`、`Core/Src/i2c.c:239–243` | USART1 RX DMA 和实际优先级、TIM6 2/0 都可能被生成器覆盖。I2C2 初始化与停用须涵盖 PB10/11 两脚，当前 DeInit 的 PB10 有重复释放，教师归并时保留两脚的对称处理 |
| `startup_stm32f407xx.s:143,154,183,186,200,204` | 启动表含 SysTick、EXTI2、USART1、EXTI15_10、TIM6_DAC、DMA2_Stream2，且有弱默认别名。执行固件审计或用 `arm-none-eabi-nm` 核对实际强实现，检查向量地址指向 |

`KeepUserCode=true`（`.ioc:244`）保护匹配到的生成区段。课程对区外代码单独盘点，发布前做一次可重复重生成并按 S00、S01、S06、S08 的相关测试回归。

### 4.2 普通运动帧与并行回零

以下中断交接对应本版的正式 Production 预设。

- 普通运动帧从 `MotionService_SubmitFrame` 起受 Safety 和 IRQ 锁存约束。TIM6 IRQ 经 HAL 回调进入 `App_ISR_OnTimerElapsed`，匹配 TIM6 后，生产路径先走 `StopForRuntimeLimit`，再调用 `MotionService_OnStepTickFromISR`。执行器用整数节拍分配三轴步数，输出走普通 `Dev_Joint_Step`。见 `app_isr_adapter.c:50–106`、`motion_service.c:38–68`、`ctrl_motion_engine.c:235–278`。
- 回零由 `SysTick_Handler` 经 `App_ISR_OnMillisecond` 进入 `Svc_HomingSequence_OnTickFromISR`。GPIO 就绪之后、仅在 `s_configured && !s_committed` 时推进；先考虑 IRQ 锁存及主循环 250 ms 心跳。板端按 STEP 拉低、方向/使能、允许轴上升沿的顺序应用输出。每轴每 1 ms 最多一步，漏 tick 不追步。App 主循环的 `Svc_Homing_Step` 在此编译分支只喂心跳。见 `app_isr_adapter.c:83–94`、`svc_homing_sequence.c:59–101`、`svc_homing.c:15–33`。
- `ROBOTARM_SEQUENTIAL_HOMING=1` 编入序列框架，`ROBOTARM_LIMIT_STATE_MODE=1` 选择 GPIO，`ROBOTARM_PARALLEL_HOMING=1` 选择三轴接触屏障和并行退回。正式两预设三项均为 ON（`CMakeLists.txt:53–62,178–185`、`CMakePresets.json` 继承链）。
- 回零完成后 `Svc_HomingSequence_HoldForCalibration` 和 100 ms 保持先锁住 STEP，主循环执行编码器置零。教师预填的 `App_Calibration_Execute` 经 `MotionService_SetClosedLoopAxisZero` 和 `Ctrl_ClosedLoop_SetAxisZero` 到 `Dev_Joint_SetFeedbackZero`，失败轴最多 3 次，重试相隔 10 ms。S09 的教师置零桥保留 Device 返回值，成功标志只由真实置零产生，闭环调节维持禁用。S09 暂存归位坐标和禁用遥操作的初始化分别由 `TEACH_GCODE_BOOT_COORD`、`TEACH_PS2_INACTIVE` 承担。S11 验证真实归位坐标，S12 和 S14 接回业务初始化。见 `app_calibration.c:12–36`、`ctrl_closed_loop.c:543–547`、`app_runtime.c:379–435,520–545`。
- 正式构建须依次完成 `Svc_HomingSequence_CommitReady`、Safety 参考/故障状态提交、恢复 READY 持有和 TIM6 启动。交接前后检查锁存，M112 可撤销轮次以及输出，M999 经完整新轮次取得新参考。S08 的 Host 可受理帧由测试显式赋予许可，S09 的 Host 可测试交接，学生板在这两步仍保持禁动。普通驱动的短高脉冲与回零位操作的脉宽均须测量，1 ms 表示调度上限。见 `svc_homing_sequence.c:147–162`、`app_runtime.c:401–435,488–498`、`bsp_robot_board.c:95–108`。

### 4.3 启动特权与运行期夹爪门禁

`App_CheckEncoderReadiness` 在驱动回读已过、三路反馈同轮有效、`Svc_HomingSequence_BootReady` 和 IRQ 门通过后才触发一次启动夹爪检查。S09 的 `TEACH_GRP_BOOT_ACK` 是 Host 可注入成功/失败的无 PWM 端口。返回失败或同期故障时，不进入回零并停机。S13 将同一入口接为 `AppHardware_TryInitGripper` → `Dev_Gripper_TryInit` → `BSP_RobotBoard_TryInitGripper` → `Drv_Servo_TryInit`。成功后才置 `s_gripper_initialized`。已成功初始化标记在同一次上电的 M999 路径中保留。启动输出许可只存在于上述路径。

运行期 `Svc_Gripper_Open/Close` 用短临界区原子检查 `MotionService_IsActionAllowed()`（Safety、运动故障、IRQ），通过后才尝试 Device 动作。HOMING 主循环中的 `Svc_Gripper_IdleStop` 负责让已经受理的启动 PWM 到 600 ms 后停止。保留设备返回错误，禁止无界重试或隐式重新授权。`ROBOTARM_STARTUP_GRIPPER_IDLE` 为仅绑定的可选分支。见 `app_runtime.c:327–376,438–450,506–509`、`app_hardware_adapter.c:48–54`、`bsp_robot_board.c:49–56`、`svc_gripper.c:11–31`、`drv_servo.c:29–151`。

## 5. 首轮试教包 S00–S02

### 教师开课前交付

- 发放独立学生副本、冻结起点的 Git 提交号以及白名单。保留 `TeachingLimitOnly` 配置和只读诊断任务，后者中的查询及限位观察合同由教师预填。保全 `RobotArm/App/` 的应用接口及现有 Host 替身。未上线的默认动作请求应在板级输出端口被拒绝。
- 让 S00 教师运输替身在 BSP 初始化时建立可用的 DMA 测试缓存及锁定端口，S01 实现前只缺真实 DMA 消费和发送能力。S01 留短行测试替身，S02 的独立用例必须因进阶成行合同未实现而失败。教师在发包前从空构建目录试跑三步，并确认红灯位置。仓库整机参考已包含最终实现，直接在其上跑测试得到的是回归结果。
- 第一次试教对照选 `TeachingLimitOnly`。`TeachingStartupAudit` 在驱动和编码器准备审计后阻断动作，使用 `AUDIT1`。独立 `ProductionLocked` 返回试验 `STARTUP8`、`MOTION_COMPILED:0`。本次学生身份测试使用 `DIAG3` 的合同。

### 学生操作清单

1. 打开教师发放的工程根目录作为 VS Code 工作区，在 PowerShell 终端核对 `git status --short --branch` 和 `git rev-parse --short HEAD`。与教师发布的提交一致后，在学生副本用 `git switch -c vibe-s00-s02` 建立自己的分支。
2. S00 运行下列命令。确认时间和停机端口通过，ARM 能构建。用此检查点作为后续差异的起点，记录编译器、测试数和镜像路径。
3. S01 先构建并运行下面第二组。记录 DMA 完整行不可用的断言位置。只实现卡中两份积木和适配端口，照原命令复跑，确认每个 DMA 回卷场景结果，再提交 S01。
4. S02 先跑下文提示词里的验收命令并记录红灯。修改限定两份文件后重复同一命令。检查点提交前执行 `git diff --name-only` 与 `git diff --check`。在 Host 上记录 `fake_enable_true_count`、`fake_timer_start_count`、`fake_gripper_open_count`、`fake_gripper_close_count`、逐轴 `fake_steps[]` 和 `fake_wave_rises[]`，都应为 0。来源 `tests/diagnostic_minimal_runtime_tests.c:45–53`。该测试注入已成行文本，字节传输由同一筛选集合中的纯成行和真实 UART BSP 用例补齐。

```powershell
# S00
cmake --preset HostTests
cmake --build --preset HostTests --target platform_time_tests stop_actuators_tests
ctest --test-dir build/HostTests --output-on-failure --no-tests=error -R '^(platform_time_tests|stop_actuators_tests|architecture_dependencies)$'
cmake --preset TeachingLimitOnly
cmake --build --preset TeachingLimitOnly

# S01 前测和复测
cmake --build --preset HostTests --target bsp_uart_tests
ctest --test-dir build/HostTests --output-on-failure --no-tests=error -R '^bsp_uart_(completed_wrap|deferred_wrap|discard_wrap|deferred_overwrite)$'
cmake --build --preset TeachingLimitOnly
```

学生交付红绿灯记录、白名单差异、ARM 构建状态与 Host 动作计数表。若现场安排已授权的只读演示，使用 115200 8N1，发一行 M115 并记录一次身份回包；实际输出电平另由教师记录。

### S02 学生 AI 提示词

以下整段可直接交给 AI。文件路径相对所发行的学生仓库根目录。

```text
完成 S02 锁定镜像的成行与 M115。先看 S01 检查点和红灯日志，再读
RobotArm/BSP/include/bsp/uart_line_reader.h、RobotArm/BSP/include/bsp/bsp_uart1_config.h、
RobotArm/App/include/app/app_serial_protocol.h、tests/uart_line_reader_tests.c、
tests/bsp_uart_tests.c、tests/diagnostic_minimal_runtime_tests.c。头文件和测试只读。

编辑白名单
1. RobotArm/BSP/src/uart_line_reader.c，补齐并冻结纯 C 实现。
2. RobotArm/App/app_serial_protocol.c，仅接通 DIAGNOSTIC 模式的 M115 分支，
   保留已有只读门禁、查询、错误和其他模式。退出临时成行和身份替身。
不改 BSP UART、GPIO、IRQ、任何头文件、.ioc、CMake、测试或 S01 冻结积木。
如果当前教师副本缺诊断查询分发合同，把阻塞和文件位置交回教师。

保持签名
void UartLineReader_Init(UartLineReader_t *reader);
bool UartLineReader_Feed(UartLineReader_t *reader, uint8_t byte, uint32_t now_ms,
                         char *out, size_t capacity);
void UartLineReader_CheckTimeout(UartLineReader_t *reader, uint32_t now_ms,
                                uint32_t timeout_ms);
void UartLineReader_Invalidate(UartLineReader_t *reader);
void AppSerial_Service(void);

UART_LINE_CAPACITY 为 256U，最大 255 个载荷字节加 NUL。
BSP_UART1_RX_BUF_SIZE 为 256U，BSP_UART1_TX_BUF_SIZE 为 1024U，
BSP_UART1_LINE_TIMEOUT_MS 为 500U。沿用无符号回绕时差，严格超过期限才超时。
行未完整返回 false，CR/LF/CRLF 结束一条非空行，CRLF 不重复回包。
允许可打印 ASCII 和 TAB。超长、NUL、其他控制字节、DMA 覆盖或半行超时，
应丢弃残行至边界，后续合法行恢复。输出容量不足时拒绝，不输出截断命令。
动作计数从 Host 假设备上的现有断言得到。只读 M115 回包含
PROTOCOL:DIAG3 MOTOR_ENABLE:LOCKED ENCODER_POLL:0，对 G1 X1 等维持拒绝。

先跑以下命令并指出预期断言，再实现与复跑。仅给出修改文件、红灯/绿灯、
命中用例和 ARM 退出码，不代为提交。
cmake --preset HostTests
cmake --build --preset HostTests --target uart_line_reader_tests bsp_uart_tests diagnostic_minimal_runtime_tests
ctest --test-dir build/HostTests --output-on-failure --no-tests=error -R '^(uart_line_reader_tests|bsp_uart_.*|diagnostic_minimal_runtime_tests)$'
cmake --preset TeachingLimitOnly
cmake --build --preset TeachingLimitOnly
git diff --name-only
git diff --check
```

### 开课拍板项

1. 正式回零课程的角度已定为 101.7°/29°/4°。教师在发布台架镜像前，按 §1.3 对齐并复验 Host 和 ELF 断言。
2. 冻结教师种子前，确定 CubeMX 区外钩子的保全做法及交付的生成版本，复验 UART RX DMA、优先级、GPIO 状态采样和启动锁。教师发包前锁定含新增参与构建文件的完整清单。
3. 明确 S15 台架的独立现场授权和空载范围。试教先以禁动 Host 和 ARM 构建签收，正式镜像的自动闭爪、并行回零及负对照在现场验收表签收。


## 附录　其余现行 C/H 文件

下表逐项补充测试、独立工具和随仓库分发的第三方源码。测试任务按测试运行，学生编辑权限由第 3 节约束。第三方交叉平台目录同时包含当前 STM32F407 未引用的文件，实际编入件以 `cmake/stm32cubemx/CMakeLists.txt` 为准。

| 文件 | 层级与职责 | 管控类别 |
| --- | --- | --- |
| `tests/action_permission_tests.c` | Host · action permission 用例 | 教师锁死合同/骨架 |
| `tests/as5600_driver_tests.c` | Host · as5600 driver 用例 | 教师锁死合同/骨架 |
| `tests/axis_binding_tests.c` | Host · axis binding 用例 | 教师锁死合同/骨架 |
| `tests/axis_limit_home_pipeline_tests.c` | Host · axis limit home pipeline 用例 | 教师锁死合同/骨架 |
| `tests/bsp_uart_tests.c` | Host · bsp uart 用例 | 教师锁死合同/骨架 |
| `tests/closed_loop_feedback_timing_tests.c` | Host · closed loop feedback timing 用例 | 教师锁死合同/骨架 |
| `tests/closed_loop_status_tests.c` | Host · closed loop status 用例 | 教师锁死合同/骨架 |
| `tests/closed_loop_teleop_completion_tests.c` | Host · closed loop teleop completion 用例 | 教师锁死合同/骨架 |
| `tests/closed_loop_tests.c` | Host · closed loop 用例 | 教师锁死合同/骨架 |
| `tests/closed_loop_window_tests.c` | Host · closed loop window 用例 | 教师锁死合同/骨架 |
| `tests/command_service_tests.c` | Host · command service 用例 | 教师锁死合同/骨架 |
| `tests/diagnostic_minimal_runtime_tests.c` | Host · diagnostic minimal runtime 用例 | 教师锁死合同/骨架 |
| `tests/diagnostic_runtime_tests.c` | Host · diagnostic runtime 用例 | 教师锁死合同/骨架 |
| `tests/feedback_protocol_tests.c` | Host · feedback protocol 用例 | 教师锁死合同/骨架 |
| `tests/firmware_integration_tests.c` | Host · firmware integration 用例 | 教师锁死合同/骨架 |
| `tests/gripper_logic_tests.c` | Host · gripper logic 用例 | 教师锁死合同/骨架 |
| `tests/gripper_protocol_tests.c` | Host · gripper protocol 用例 | 教师锁死合同/骨架 |
| `tests/gripper_result_chain_tests.c` | Host · gripper result chain 用例 | 教师锁死合同/骨架 |
| `tests/hal_stubs/direction/main.h` | Host · HAL 替身合同（direction） | 教师锁死合同/骨架 |
| `tests/hal_stubs/i2c_recovery/i2c.h` | Host · HAL 替身合同（i2c_recovery） | 教师锁死合同/骨架 |
| `tests/hal_stubs/i2c_recovery/main.h` | Host · HAL 替身合同（i2c_recovery） | 教师锁死合同/骨架 |
| `tests/hal_stubs/i2c_recovery/stm32f4xx_hal.h` | Host · HAL 替身合同（i2c_recovery） | 教师锁死合同/骨架 |
| `tests/hal_stubs/isr/main.h` | Host · HAL 替身合同（isr） | 教师锁死合同/骨架 |
| `tests/hal_stubs/limit_irq/main.h` | Host · HAL 替身合同（limit_irq） | 教师锁死合同/骨架 |
| `tests/hal_stubs/limit_trace/main.h` | Host · HAL 替身合同（limit_trace） | 教师锁死合同/骨架 |
| `tests/hal_stubs/m1_deadline/main.h` | Host · HAL 替身合同（m1_deadline） | 教师锁死合同/骨架 |
| `tests/hal_stubs/servo/main.h` | Host · HAL 替身合同（servo） | 教师锁死合同/骨架 |
| `tests/hal_stubs/stop/main.h` | Host · HAL 替身合同（stop） | 教师锁死合同/骨架 |
| `tests/hal_stubs/stop/tim.h` | Host · HAL 替身合同（stop） | 教师锁死合同/骨架 |
| `tests/hal_stubs/stop/usart.h` | Host · HAL 替身合同（stop） | 教师锁死合同/骨架 |
| `tests/hal_stubs/time/main.h` | Host · HAL 替身合同（time） | 教师锁死合同/骨架 |
| `tests/hal_stubs/tmc/main.h` | Host · HAL 替身合同（tmc） | 教师锁死合同/骨架 |
| `tests/hal_stubs/usart.h` | Host · HAL 替身合同（usart.h） | 教师锁死合同/骨架 |
| `tests/hardware_adapter_tests.c` | Host · hardware adapter 用例 | 教师锁死合同/骨架 |
| `tests/home_contact_cancel_tests.c` | Host · home contact cancel 用例 | 教师锁死合同/骨架 |
| `tests/home_other_input_tests.c` | Host · home other input 用例 | 教师锁死合同/骨架 |
| `tests/home_seek500_tests.c` | Host · home seek500 用例 | 教师锁死合同/骨架 |
| `tests/home_sequence_pause_tests.c` | Host · home sequence pause 用例 | 教师锁死合同/骨架 |
| `tests/home_sequence_speed_tests.c` | Host · home sequence speed 用例 | 教师锁死合同/骨架 |
| `tests/home_sequence_tests.c` | Host · home sequence 用例 | 教师锁死合同/骨架 |
| `tests/home1000_tests.c` | Host · home1000 用例 | 教师锁死合同/骨架 |
| `tests/homing_gpio_contract_tests.c` | Host · homing gpio contract 用例 | 教师锁死合同/骨架 |
| `tests/homing_logging_tests.c` | Host · homing logging 用例 | 教师锁死合同/骨架 |
| `tests/homing_service_tests.c` | Host · homing service 用例 | 教师锁死合同/骨架 |
| `tests/i2c_recovery_tests.c` | Host · i2c recovery 用例 | 教师锁死合同/骨架 |
| `tests/irq_axis_evidence_tests.c` | Host · irq axis evidence 用例 | 教师锁死合同/骨架 |
| `tests/irq_evidence_tests.c` | Host · irq evidence 用例 | 教师锁死合同/骨架 |
| `tests/irq_guard_integration_tests.c` | Host · irq guard integration 用例 | 教师锁死合同/骨架 |
| `tests/isr_routing_tests.c` | Host · isr routing 用例 | 教师锁死合同/骨架 |
| `tests/limit_indicator_tick_tests.c` | Host · limit indicator tick 用例 | 教师锁死合同/骨架 |
| `tests/limit_irq_guard_tests.c` | Host · limit irq guard 用例 | 教师锁死合同/骨架 |
| `tests/limit_irq_port_tests.c` | Host · limit irq port 用例 | 教师锁死合同/骨架 |
| `tests/limit_observation_tests.c` | Host · limit observation 用例 | 教师锁死合同/骨架 |
| `tests/limit_policy_comparison_tests.c` | Host · limit policy comparison 用例 | 教师锁死合同/骨架 |
| `tests/limit_probe_console_tests.c` | Host · limit probe console 用例 | 教师锁死合同/骨架 |
| `tests/limit_probe_logic_tests.c` | Host · limit probe logic 用例 | 教师锁死合同/骨架 |
| `tests/limit_seek_extended_tests.c` | Host · limit seek extended 用例 | 教师锁死合同/骨架 |
| `tests/limit_seek_logic_tests.c` | Host · limit seek logic 用例 | 教师锁死合同/骨架 |
| `tests/limit_state_filter_tests.c` | Host · limit state filter 用例 | 教师锁死合同/骨架 |
| `tests/limit_state_port_tests.c` | Host · limit state port 用例 | 教师锁死合同/骨架 |
| `tests/limit_vector_trace_tests.c` | Host · limit vector trace 用例 | 教师锁死合同/骨架 |
| `tests/logic_tests.c` | Host · logic 用例 | 教师锁死合同/骨架 |
| `tests/m1_deadline_tests.c` | Host · m1 deadline 用例 | 教师锁死合同/骨架 |
| `tests/m1_homing_logic_tests.c` | Host · m1 homing logic 用例 | 教师锁死合同/骨架 |
| `tests/m1_homing_rx_tests.c` | Host · m1 homing rx 用例 | 教师锁死合同/骨架 |
| `tests/m1_limits_tests.c` | Host · m1 limits 用例 | 教师锁死合同/骨架 |
| `tests/m1_reference_tests.c` | Host · m1 reference 用例 | 教师锁死合同/骨架 |
| `tests/mode_switch_visibility_tests.c` | Host · mode switch visibility 用例 | 教师锁死合同/骨架 |
| `tests/motion_engine_tests.c` | Host · motion engine 用例 | 教师锁死合同/骨架 |
| `tests/parallel_homing_tests.c` | Host · parallel homing 用例 | 教师锁死合同/骨架 |
| `tests/parallel_runtime_tests.c` | Host · parallel runtime 用例 | 教师锁死合同/骨架 |
| `tests/planning_service_tests.c` | Host · planning service 用例 | 教师锁死合同/骨架 |
| `tests/platform_time_tests.c` | Host · platform time 用例 | 教师锁死合同/骨架 |
| `tests/ps2_diagnostic_query_tests.c` | Host · ps2 diagnostic query 用例 | 教师锁死合同/骨架 |
| `tests/ps2_driver_tests.c` | Host · ps2 driver 用例 | 教师锁死合同/骨架 |
| `tests/ready_hold_tests.c` | Host · ready hold 用例 | 教师锁死合同/骨架 |
| `tests/real_hal_limit_irq_tests.c` | Host · real hal limit irq 用例 | 教师锁死合同/骨架 |
| `tests/reference_protocol_tests.c` | Host · reference protocol 用例 | 教师锁死合同/骨架 |
| `tests/safety_service_tests.c` | Host · safety service 用例 | 教师锁死合同/骨架 |
| `tests/seek_contact_irq_tests.c` | Host · seek contact irq 用例 | 教师锁死合同/骨架 |
| `tests/seek_input_hold_tests.c` | Host · seek input hold 用例 | 教师锁死合同/骨架 |
| `tests/seek_result_tests.c` | Host · seek result 用例 | 教师锁死合同/骨架 |
| `tests/sequence_runtime_tests.c` | Host · sequence runtime 用例 | 教师锁死合同/骨架 |
| `tests/serial_reply_flow_tests.c` | Host · serial reply flow 用例 | 教师锁死合同/骨架 |
| `tests/servo_driver_tests.c` | Host · servo driver 用例 | 教师锁死合同/骨架 |
| `tests/startup_audit_tests.c` | Host · startup audit 用例 | 教师锁死合同/骨架 |
| `tests/startup_driver_tests.c` | Host · startup driver 用例 | 教师锁死合同/骨架 |
| `tests/startup_feedback_irq_tests.c` | Host · startup feedback irq 用例 | 教师锁死合同/骨架 |
| `tests/startup_finalize_irq_tests.c` | Host · startup finalize irq 用例 | 教师锁死合同/骨架 |
| `tests/startup_irq_boundary_tests.c` | Host · startup irq boundary 用例 | 教师锁死合同/骨架 |
| `tests/startup_readiness_tests.c` | Host · startup readiness 用例 | 教师锁死合同/骨架 |
| `tests/startup_trial_app_tests.c` | Host · startup trial app 用例 | 教师锁死合同/骨架 |
| `tests/startup_trial_logic_tests.c` | Host · startup trial logic 用例 | 教师锁死合同/骨架 |
| `tests/state_and_teleop_tests.c` | Host · state and teleop 用例 | 教师锁死合同/骨架 |
| `tests/state_feedback_tests.c` | Host · state feedback 用例 | 教师锁死合同/骨架 |
| `tests/stop_actuators_tests.c` | Host · stop actuators 用例 | 教师锁死合同/骨架 |
| `tests/support/fake_hardware.c` | Host · 设备、时基或传输替身（fake_hardware） | 教师锁死合同/骨架 |
| `tests/support/fake_hardware.h` | Host · 设备、时基或传输替身（fake_hardware） | 教师锁死合同/骨架 |
| `tests/support/fake_robot.c` | Host · 设备、时基或传输替身（fake_robot） | 教师锁死合同/骨架 |
| `tests/support/fake_robot.h` | Host · 设备、时基或传输替身（fake_robot） | 教师锁死合同/骨架 |
| `tests/teleop_continuity_tests.c` | Host · teleop continuity 用例 | 教师锁死合同/骨架 |
| `tests/teleop_coordinate_handoff_tests.c` | Host · teleop coordinate handoff 用例 | 教师锁死合同/骨架 |
| `tests/teleop_lifecycle_tests.c` | Host · teleop lifecycle 用例 | 教师锁死合同/骨架 |
| `tests/tmc_bench_tests.c` | Host · tmc bench 用例 | 教师锁死合同/骨架 |
| `tests/tmc2209_bus_model_tests.c` | Host · tmc2209 bus model 用例 | 教师锁死合同/骨架 |
| `tests/tmc2209_driver_tests.c` | Host · tmc2209 driver 用例 | 教师锁死合同/骨架 |
| `tests/uart_line_reader_tests.c` | Host · uart line reader 用例 | 教师锁死合同/骨架 |
| `tests/waveform_encoder_mask_tests.c` | Host · waveform encoder mask 用例 | 教师锁死合同/骨架 |
| `tests/waveform_runtime_tests.c` | Host · waveform runtime 用例 | 教师锁死合同/骨架 |
| `tests/waveform_service_tests.c` | Host · waveform service 用例 | 教师锁死合同/骨架 |

| 文件 | 层级与职责 | 管控类别 |
| --- | --- | --- |
| `tools/gripper_test/gripper_logic.c` | Tools · gripper_test 的 gripper_logic 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/gripper_test/gripper_logic.h` | Tools · gripper_test 的 gripper_logic 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/gripper_test/main.c` | Tools · gripper_test 的 main 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/host/offline_plan_check.c` | Tools · host 的 offline_plan_check 模块 | 诊断或历史兼容分支，教师用于锁定、离线验证或 S15 对照 |
| `tools/limit_probe/main.c` | Tools · limit_probe 的 main 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/limit_probe/probe_console.c` | Tools · limit_probe 的 probe_console 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/limit_probe/probe_console.h` | Tools · limit_probe 的 probe_console 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/limit_probe/probe_logic.c` | Tools · limit_probe 的 probe_logic 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/limit_probe/probe_logic.h` | Tools · limit_probe 的 probe_logic 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/limit_seek_trial/boot_contact.c` | Tools · limit_seek_trial 的 boot_contact 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/limit_seek_trial/boot_contact.h` | Tools · limit_seek_trial 的 boot_contact 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/limit_seek_trial/contact_gate.c` | Tools · limit_seek_trial 的 contact_gate 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/limit_seek_trial/contact_gate.h` | Tools · limit_seek_trial 的 contact_gate 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/limit_seek_trial/guard.c` | Tools · limit_seek_trial 的 guard 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/limit_seek_trial/input_hold.c` | Tools · limit_seek_trial 的 input_hold 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/limit_seek_trial/input_hold.h` | Tools · limit_seek_trial 的 input_hold 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/limit_seek_trial/seek_logic.c` | Tools · limit_seek_trial 的 seek_logic 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/limit_seek_trial/seek_logic.h` | Tools · limit_seek_trial 的 seek_logic 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/limit_seek_trial/seek_result.c` | Tools · limit_seek_trial 的 seek_result 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/limit_seek_trial/seek_result.h` | Tools · limit_seek_trial 的 seek_result 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/m1_homing/axis_binding.c` | Tools · m1_homing 的 axis_binding 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/m1_homing/axis_binding.h` | Tools · m1_homing 的 axis_binding 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/m1_homing/m1_deadline.c` | Tools · m1_homing 的 m1_deadline 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/m1_homing/m1_deadline.h` | Tools · m1_homing 的 m1_deadline 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/m1_homing/m1_limits.c` | Tools · m1_homing 的 m1_limits 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/m1_homing/m1_limits.h` | Tools · m1_homing 的 m1_limits 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/m1_homing/m1_logic.c` | Tools · m1_homing 的 m1_logic 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/m1_homing/m1_logic.h` | Tools · m1_homing 的 m1_logic 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/m1_homing/m1_reference.c` | Tools · m1_homing 的 m1_reference 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/m1_homing/m1_reference.h` | Tools · m1_homing 的 m1_reference 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/m1_homing/m1_rx.c` | Tools · m1_homing 的 m1_rx 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/m1_homing/m1_rx.h` | Tools · m1_homing 的 m1_rx 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/m1_homing/main.c` | Tools · m1_homing 的 main 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/startup_audit/guard.c` | Tools · startup_audit 的 guard 模块 | 诊断或历史兼容分支，教师用于锁定、离线验证或 S15 对照 |
| `tools/startup_audit/irq_evidence.c` | Tools · startup_audit 的 irq_evidence 模块 | 诊断或历史兼容分支，教师用于锁定、离线验证或 S15 对照 |
| `tools/startup_audit/irq_evidence.h` | Tools · startup_audit 的 irq_evidence 模块 | 诊断或历史兼容分支，教师用于锁定、离线验证或 S15 对照 |
| `tools/startup_trial/guard.c` | Tools · startup_trial 的 guard 模块 | 诊断或历史兼容分支，教师用于锁定、离线验证或 S15 对照 |
| `tools/startup_trial/ready_hold.c` | Tools · startup_trial 的 ready_hold 模块 | 诊断或历史兼容分支，教师用于锁定、离线验证或 S15 对照 |
| `tools/startup_trial/ready_hold.h` | Tools · startup_trial 的 ready_hold 模块 | 诊断或历史兼容分支，教师用于锁定、离线验证或 S15 对照 |
| `tools/startup_trial/trial_logic.c` | Tools · startup_trial 的 trial_logic 模块 | 诊断或历史兼容分支，教师用于锁定、离线验证或 S15 对照 |
| `tools/startup_trial/trial_logic.h` | Tools · startup_trial 的 trial_logic 模块 | 诊断或历史兼容分支，教师用于锁定、离线验证或 S15 对照 |
| `tools/tmc_bench/bench_logic.c` | Tools · tmc_bench 的 bench_logic 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/tmc_bench/bench_logic.h` | Tools · tmc_bench 的 bench_logic 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/tmc_bench/legacy_bridge.c` | Tools · tmc_bench 的 legacy_bridge 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/tmc_bench/legacy_bridge.h` | Tools · tmc_bench 的 legacy_bridge 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/tmc_bench/legacy_original/include/bsp_driver_config.h` | Tools · tmc_bench 的 bsp_driver_config 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/tmc_bench/legacy_original/include/driver/drv_stepper.h` | Tools · tmc_bench 的 drv_stepper 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/tmc_bench/legacy_original/include/driver/drv_tmc2209.h` | Tools · tmc_bench 的 drv_tmc2209 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/tmc_bench/legacy_original/src/drv_stepper.c` | Tools · tmc_bench 的 drv_stepper 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/tmc_bench/legacy_original/src/drv_tmc2209.c` | Tools · tmc_bench 的 drv_tmc2209 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/tmc_bench/main.c` | Tools · tmc_bench 的 main 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/tmc_config_probe/main.c` | Tools · tmc_config_probe 的 main 模块 | 诊断或历史兼容分支，主线学生免改 |
| `tools/uart_boot_probe/main.c` | Tools · uart_boot_probe 的 main 模块 | 诊断或历史兼容分支，主线学生免改 |

<details><summary>Drivers 第三方源码与头文件</summary>

| 文件 | 层级与职责 | 管控类别 |
| --- | --- | --- |
| `Drivers/CMSIS/Core_A/Include/cmsis_armcc.h` | CMSIS · Core_A 中的 cmsis_armcc 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core_A/Include/cmsis_armclang.h` | CMSIS · Core_A 中的 cmsis_armclang 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core_A/Include/cmsis_compiler.h` | CMSIS · Core_A 中的 cmsis_compiler 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core_A/Include/cmsis_cp15.h` | CMSIS · Core_A 中的 cmsis_cp15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core_A/Include/cmsis_gcc.h` | CMSIS · Core_A 中的 cmsis_gcc 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core_A/Include/cmsis_iccarm.h` | CMSIS · Core_A 中的 cmsis_iccarm 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core_A/Include/core_ca.h` | CMSIS · Core_A 中的 core_ca 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core_A/Include/irq_ctrl.h` | CMSIS · Core_A 中的 irq_ctrl 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core_A/Source/irq_ctrl_gic.c` | CMSIS · Core_A 中的 irq_ctrl_gic 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/cachel1_armv7.h` | CMSIS · Core 中的 cachel1_armv7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/cmsis_armcc.h` | CMSIS · Core 中的 cmsis_armcc 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/cmsis_armclang_ltm.h` | CMSIS · Core 中的 cmsis_armclang_ltm 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/cmsis_armclang.h` | CMSIS · Core 中的 cmsis_armclang 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/cmsis_compiler.h` | CMSIS · Core 中的 cmsis_compiler 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/cmsis_gcc.h` | CMSIS · Core 中的 cmsis_gcc 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/cmsis_iccarm.h` | CMSIS · Core 中的 cmsis_iccarm 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/cmsis_version.h` | CMSIS · Core 中的 cmsis_version 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_armv81mml.h` | CMSIS · Core 中的 core_armv81mml 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_armv8mbl.h` | CMSIS · Core 中的 core_armv8mbl 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_armv8mml.h` | CMSIS · Core 中的 core_armv8mml 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_cm0.h` | CMSIS · Core 中的 core_cm0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_cm0plus.h` | CMSIS · Core 中的 core_cm0plus 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_cm1.h` | CMSIS · Core 中的 core_cm1 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_cm23.h` | CMSIS · Core 中的 core_cm23 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_cm3.h` | CMSIS · Core 中的 core_cm3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_cm33.h` | CMSIS · Core 中的 core_cm33 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_cm35p.h` | CMSIS · Core 中的 core_cm35p 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_cm4.h` | CMSIS · Core 中的 core_cm4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_cm55.h` | CMSIS · Core 中的 core_cm55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_cm7.h` | CMSIS · Core 中的 core_cm7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_cm85.h` | CMSIS · Core 中的 core_cm85 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_sc000.h` | CMSIS · Core 中的 core_sc000 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_sc300.h` | CMSIS · Core 中的 core_sc300 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/core_starmc1.h` | CMSIS · Core 中的 core_starmc1 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/mpu_armv7.h` | CMSIS · Core 中的 mpu_armv7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/mpu_armv8.h` | CMSIS · Core 中的 mpu_armv8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/pac_armv81.h` | CMSIS · Core 中的 pac_armv81 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/pmu_armv8.h` | CMSIS · Core 中的 pmu_armv8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Include/tz_context.h` | CMSIS · Core 中的 tz_context 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Template/ARMv8-M/main_s.c` | CMSIS · Core 中的 main_s 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Core/Template/ARMv8-M/tz_context.c` | CMSIS · Core 中的 tz_context 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DAP/Firmware/Config/DAP_config.h` | CMSIS · DAP 中的 DAP_config 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DAP/Firmware/Include/DAP.h` | CMSIS · DAP 中的 DAP 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DAP/Firmware/Source/DAP_vendor.c` | CMSIS · DAP 中的 DAP_vendor 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DAP/Firmware/Source/DAP.c` | CMSIS · DAP 中的 DAP 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DAP/Firmware/Source/JTAG_DP.c` | CMSIS · DAP 中的 JTAG_DP 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DAP/Firmware/Source/SW_DP.c` | CMSIS · DAP 中的 SW_DP 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DAP/Firmware/Source/SWO.c` | CMSIS · DAP 中的 SWO 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DAP/Firmware/Source/UART.c` | CMSIS · DAP 中的 UART 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f401xc.h` | CMSIS Device · stm32f401xc 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f401xe.h` | CMSIS Device · stm32f401xe 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f405xx.h` | CMSIS Device · stm32f405xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h` | CMSIS Device · stm32f407xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f410cx.h` | CMSIS Device · stm32f410cx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f410rx.h` | CMSIS Device · stm32f410rx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f410tx.h` | CMSIS Device · stm32f410tx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f411xe.h` | CMSIS Device · stm32f411xe 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f412cx.h` | CMSIS Device · stm32f412cx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f412rx.h` | CMSIS Device · stm32f412rx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f412vx.h` | CMSIS Device · stm32f412vx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f412zx.h` | CMSIS Device · stm32f412zx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h` | CMSIS Device · stm32f413xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f415xx.h` | CMSIS Device · stm32f415xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f417xx.h` | CMSIS Device · stm32f417xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f423xx.h` | CMSIS Device · stm32f423xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f427xx.h` | CMSIS Device · stm32f427xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f429xx.h` | CMSIS Device · stm32f429xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f437xx.h` | CMSIS Device · stm32f437xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f439xx.h` | CMSIS Device · stm32f439xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f446xx.h` | CMSIS Device · stm32f446xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f469xx.h` | CMSIS Device · stm32f469xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f479xx.h` | CMSIS Device · stm32f479xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h` | CMSIS Device · stm32f4xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Include/system_stm32f4xx.h` | CMSIS Device · system_stm32f4xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c` | CMSIS Device · system_stm32f4xx 芯片声明或平台实现 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/ComputeLibrary/Include/NEMath.h` | CMSIS · DSP 中的 NEMath 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/ComputeLibrary/Source/arm_cl_tables.c` | CMSIS · DSP 中的 arm_cl_tables 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_bayes_example/arm_bayes_example_f32.c` | CMSIS · DSP 中的 arm_bayes_example_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_bayes_example/RTE/Device/ARMCM0/startup_ARMCM0.c` | CMSIS · DSP 中的 startup_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_bayes_example/RTE/Device/ARMCM0/system_ARMCM0.c` | CMSIS · DSP 中的 system_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_bayes_example/RTE/Device/ARMCM3/startup_ARMCM3.c` | CMSIS · DSP 中的 startup_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_bayes_example/RTE/Device/ARMCM3/system_ARMCM3.c` | CMSIS · DSP 中的 system_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_bayes_example/RTE/Device/ARMCM4_FP/startup_ARMCM4.c` | CMSIS · DSP 中的 startup_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_bayes_example/RTE/Device/ARMCM4_FP/system_ARMCM4.c` | CMSIS · DSP 中的 system_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_bayes_example/RTE/Device/ARMCM55/startup_ARMCM55.c` | CMSIS · DSP 中的 startup_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_bayes_example/RTE/Device/ARMCM55/system_ARMCM55.c` | CMSIS · DSP 中的 system_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_bayes_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.c` | CMSIS · DSP 中的 startup_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_bayes_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c` | CMSIS · DSP 中的 system_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/arm_class_marks_example_f32.c` | CMSIS · DSP 中的 arm_class_marks_example_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM0/startup_ARMCM0.c` | CMSIS · DSP 中的 startup_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM0/system_ARMCM0.c` | CMSIS · DSP 中的 system_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM3/startup_ARMCM3.c` | CMSIS · DSP 中的 startup_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM3/system_ARMCM3.c` | CMSIS · DSP 中的 system_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM4_FP/startup_ARMCM4.c` | CMSIS · DSP 中的 startup_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM4_FP/system_ARMCM4.c` | CMSIS · DSP 中的 system_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM55/startup_ARMCM55.c` | CMSIS · DSP 中的 startup_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM55/system_ARMCM55.c` | CMSIS · DSP 中的 system_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.c` | CMSIS · DSP 中的 startup_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c` | CMSIS · DSP 中的 system_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_convolution_example/arm_convolution_example_f32.c` | CMSIS · DSP 中的 arm_convolution_example_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_convolution_example/math_helper.c` | CMSIS · DSP 中的 math_helper 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_convolution_example/math_helper.h` | CMSIS · DSP 中的 math_helper 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_convolution_example/RTE/Device/ARMCM0/startup_ARMCM0.c` | CMSIS · DSP 中的 startup_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_convolution_example/RTE/Device/ARMCM0/system_ARMCM0.c` | CMSIS · DSP 中的 system_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_convolution_example/RTE/Device/ARMCM3/startup_ARMCM3.c` | CMSIS · DSP 中的 startup_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_convolution_example/RTE/Device/ARMCM3/system_ARMCM3.c` | CMSIS · DSP 中的 system_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_convolution_example/RTE/Device/ARMCM4_FP/startup_ARMCM4.c` | CMSIS · DSP 中的 startup_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_convolution_example/RTE/Device/ARMCM4_FP/system_ARMCM4.c` | CMSIS · DSP 中的 system_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_convolution_example/RTE/Device/ARMCM55/startup_ARMCM55.c` | CMSIS · DSP 中的 startup_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_convolution_example/RTE/Device/ARMCM55/system_ARMCM55.c` | CMSIS · DSP 中的 system_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_convolution_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.c` | CMSIS · DSP 中的 startup_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_convolution_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c` | CMSIS · DSP 中的 system_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/arm_dotproduct_example_f32.c` | CMSIS · DSP 中的 arm_dotproduct_example_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM0/startup_ARMCM0.c` | CMSIS · DSP 中的 startup_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM0/system_ARMCM0.c` | CMSIS · DSP 中的 system_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM3/startup_ARMCM3.c` | CMSIS · DSP 中的 startup_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM3/system_ARMCM3.c` | CMSIS · DSP 中的 system_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM4_FP/startup_ARMCM4.c` | CMSIS · DSP 中的 startup_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM4_FP/system_ARMCM4.c` | CMSIS · DSP 中的 system_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/startup_ARMCM55.c` | CMSIS · DSP 中的 startup_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM55/system_ARMCM55.c` | CMSIS · DSP 中的 system_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.c` | CMSIS · DSP 中的 startup_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_dotproduct_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c` | CMSIS · DSP 中的 system_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_data.c` | CMSIS · DSP 中的 arm_fft_bin_data 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/arm_fft_bin_example_f32.c` | CMSIS · DSP 中的 arm_fft_bin_example_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/RTE/Device/ARMCM0/startup_ARMCM0.c` | CMSIS · DSP 中的 startup_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/RTE/Device/ARMCM0/system_ARMCM0.c` | CMSIS · DSP 中的 system_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/RTE/Device/ARMCM3/startup_ARMCM3.c` | CMSIS · DSP 中的 startup_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/RTE/Device/ARMCM3/system_ARMCM3.c` | CMSIS · DSP 中的 system_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/RTE/Device/ARMCM4_FP/startup_ARMCM4.c` | CMSIS · DSP 中的 startup_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/RTE/Device/ARMCM4_FP/system_ARMCM4.c` | CMSIS · DSP 中的 system_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/RTE/Device/ARMCM55/startup_ARMCM55.c` | CMSIS · DSP 中的 startup_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/RTE/Device/ARMCM55/system_ARMCM55.c` | CMSIS · DSP 中的 system_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.c` | CMSIS · DSP 中的 startup_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fft_bin_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c` | CMSIS · DSP 中的 system_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fir_example/arm_fir_data.c` | CMSIS · DSP 中的 arm_fir_data 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fir_example/arm_fir_example_f32.c` | CMSIS · DSP 中的 arm_fir_example_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fir_example/math_helper.c` | CMSIS · DSP 中的 math_helper 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fir_example/math_helper.h` | CMSIS · DSP 中的 math_helper 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fir_example/RTE/Device/ARMCM0/startup_ARMCM0.c` | CMSIS · DSP 中的 startup_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fir_example/RTE/Device/ARMCM0/system_ARMCM0.c` | CMSIS · DSP 中的 system_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fir_example/RTE/Device/ARMCM3/startup_ARMCM3.c` | CMSIS · DSP 中的 startup_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fir_example/RTE/Device/ARMCM3/system_ARMCM3.c` | CMSIS · DSP 中的 system_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fir_example/RTE/Device/ARMCM4_FP/startup_ARMCM4.c` | CMSIS · DSP 中的 startup_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fir_example/RTE/Device/ARMCM4_FP/system_ARMCM4.c` | CMSIS · DSP 中的 system_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fir_example/RTE/Device/ARMCM55/startup_ARMCM55.c` | CMSIS · DSP 中的 startup_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fir_example/RTE/Device/ARMCM55/system_ARMCM55.c` | CMSIS · DSP 中的 system_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fir_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.c` | CMSIS · DSP 中的 startup_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_fir_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c` | CMSIS · DSP 中的 system_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/arm_graphic_equalizer_data.c` | CMSIS · DSP 中的 arm_graphic_equalizer_data 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/arm_graphic_equalizer_example_q31.c` | CMSIS · DSP 中的 arm_graphic_equalizer_example_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/math_helper.c` | CMSIS · DSP 中的 math_helper 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/math_helper.h` | CMSIS · DSP 中的 math_helper 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM0/startup_ARMCM0.c` | CMSIS · DSP 中的 startup_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM0/system_ARMCM0.c` | CMSIS · DSP 中的 system_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/startup_ARMCM3.c` | CMSIS · DSP 中的 startup_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM3/system_ARMCM3.c` | CMSIS · DSP 中的 system_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM4_FP/startup_ARMCM4.c` | CMSIS · DSP 中的 startup_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM4_FP/system_ARMCM4.c` | CMSIS · DSP 中的 system_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM55/startup_ARMCM55.c` | CMSIS · DSP 中的 startup_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM55/system_ARMCM55.c` | CMSIS · DSP 中的 system_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.c` | CMSIS · DSP 中的 startup_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_graphic_equalizer_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c` | CMSIS · DSP 中的 system_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_linear_interp_example/arm_linear_interp_data.c` | CMSIS · DSP 中的 arm_linear_interp_data 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_linear_interp_example/arm_linear_interp_example_f32.c` | CMSIS · DSP 中的 arm_linear_interp_example_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_linear_interp_example/math_helper.c` | CMSIS · DSP 中的 math_helper 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_linear_interp_example/math_helper.h` | CMSIS · DSP 中的 math_helper 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_linear_interp_example/RTE/Device/ARMCM0/startup_ARMCM0.c` | CMSIS · DSP 中的 startup_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_linear_interp_example/RTE/Device/ARMCM0/system_ARMCM0.c` | CMSIS · DSP 中的 system_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_linear_interp_example/RTE/Device/ARMCM3/startup_ARMCM3.c` | CMSIS · DSP 中的 startup_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_linear_interp_example/RTE/Device/ARMCM3/system_ARMCM3.c` | CMSIS · DSP 中的 system_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_linear_interp_example/RTE/Device/ARMCM4_FP/startup_ARMCM4.c` | CMSIS · DSP 中的 startup_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_linear_interp_example/RTE/Device/ARMCM4_FP/system_ARMCM4.c` | CMSIS · DSP 中的 system_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_linear_interp_example/RTE/Device/ARMCM55/startup_ARMCM55.c` | CMSIS · DSP 中的 startup_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_linear_interp_example/RTE/Device/ARMCM55/system_ARMCM55.c` | CMSIS · DSP 中的 system_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_linear_interp_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.c` | CMSIS · DSP 中的 startup_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_linear_interp_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c` | CMSIS · DSP 中的 system_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_matrix_example/arm_matrix_example_f32.c` | CMSIS · DSP 中的 arm_matrix_example_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_matrix_example/math_helper.c` | CMSIS · DSP 中的 math_helper 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_matrix_example/math_helper.h` | CMSIS · DSP 中的 math_helper 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_matrix_example/RTE/Device/ARMCM0/startup_ARMCM0.c` | CMSIS · DSP 中的 startup_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_matrix_example/RTE/Device/ARMCM0/system_ARMCM0.c` | CMSIS · DSP 中的 system_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_matrix_example/RTE/Device/ARMCM3/startup_ARMCM3.c` | CMSIS · DSP 中的 startup_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_matrix_example/RTE/Device/ARMCM3/system_ARMCM3.c` | CMSIS · DSP 中的 system_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_matrix_example/RTE/Device/ARMCM4_FP/startup_ARMCM4.c` | CMSIS · DSP 中的 startup_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_matrix_example/RTE/Device/ARMCM4_FP/system_ARMCM4.c` | CMSIS · DSP 中的 system_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_matrix_example/RTE/Device/ARMCM55/startup_ARMCM55.c` | CMSIS · DSP 中的 startup_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_matrix_example/RTE/Device/ARMCM55/system_ARMCM55.c` | CMSIS · DSP 中的 system_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_matrix_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.c` | CMSIS · DSP 中的 startup_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_matrix_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c` | CMSIS · DSP 中的 system_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_signal_converge_example/arm_signal_converge_data.c` | CMSIS · DSP 中的 arm_signal_converge_data 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_signal_converge_example/arm_signal_converge_example_f32.c` | CMSIS · DSP 中的 arm_signal_converge_example_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_signal_converge_example/math_helper.c` | CMSIS · DSP 中的 math_helper 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_signal_converge_example/math_helper.h` | CMSIS · DSP 中的 math_helper 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_signal_converge_example/RTE/Device/ARMCM0/startup_ARMCM0.c` | CMSIS · DSP 中的 startup_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_signal_converge_example/RTE/Device/ARMCM0/system_ARMCM0.c` | CMSIS · DSP 中的 system_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_signal_converge_example/RTE/Device/ARMCM3/startup_ARMCM3.c` | CMSIS · DSP 中的 startup_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_signal_converge_example/RTE/Device/ARMCM3/system_ARMCM3.c` | CMSIS · DSP 中的 system_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_signal_converge_example/RTE/Device/ARMCM4_FP/startup_ARMCM4.c` | CMSIS · DSP 中的 startup_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_signal_converge_example/RTE/Device/ARMCM4_FP/system_ARMCM4.c` | CMSIS · DSP 中的 system_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_signal_converge_example/RTE/Device/ARMCM55/startup_ARMCM55.c` | CMSIS · DSP 中的 startup_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_signal_converge_example/RTE/Device/ARMCM55/system_ARMCM55.c` | CMSIS · DSP 中的 system_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_signal_converge_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.c` | CMSIS · DSP 中的 startup_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_signal_converge_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c` | CMSIS · DSP 中的 system_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/arm_sin_cos_example_f32.c` | CMSIS · DSP 中的 arm_sin_cos_example_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM0/startup_ARMCM0.c` | CMSIS · DSP 中的 startup_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM0/system_ARMCM0.c` | CMSIS · DSP 中的 system_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM3/startup_ARMCM3.c` | CMSIS · DSP 中的 startup_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM3/system_ARMCM3.c` | CMSIS · DSP 中的 system_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM4_FP/startup_ARMCM4.c` | CMSIS · DSP 中的 startup_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM4_FP/system_ARMCM4.c` | CMSIS · DSP 中的 system_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM55/startup_ARMCM55.c` | CMSIS · DSP 中的 startup_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM55/system_ARMCM55.c` | CMSIS · DSP 中的 system_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.c` | CMSIS · DSP 中的 startup_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_sin_cos_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c` | CMSIS · DSP 中的 system_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_svm_example/arm_svm_example_f32.c` | CMSIS · DSP 中的 arm_svm_example_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_svm_example/RTE/Device/ARMCM0/startup_ARMCM0.c` | CMSIS · DSP 中的 startup_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_svm_example/RTE/Device/ARMCM0/system_ARMCM0.c` | CMSIS · DSP 中的 system_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_svm_example/RTE/Device/ARMCM3/startup_ARMCM3.c` | CMSIS · DSP 中的 startup_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_svm_example/RTE/Device/ARMCM3/system_ARMCM3.c` | CMSIS · DSP 中的 system_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_svm_example/RTE/Device/ARMCM4_FP/startup_ARMCM4.c` | CMSIS · DSP 中的 startup_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_svm_example/RTE/Device/ARMCM4_FP/system_ARMCM4.c` | CMSIS · DSP 中的 system_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_svm_example/RTE/Device/ARMCM55/startup_ARMCM55.c` | CMSIS · DSP 中的 startup_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_svm_example/RTE/Device/ARMCM55/system_ARMCM55.c` | CMSIS · DSP 中的 system_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_svm_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.c` | CMSIS · DSP 中的 startup_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_svm_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c` | CMSIS · DSP 中的 system_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/arm_variance_example_f32.c` | CMSIS · DSP 中的 arm_variance_example_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/RTE/Device/ARMCM0/startup_ARMCM0.c` | CMSIS · DSP 中的 startup_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/RTE/Device/ARMCM0/system_ARMCM0.c` | CMSIS · DSP 中的 system_ARMCM0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/RTE/Device/ARMCM3/startup_ARMCM3.c` | CMSIS · DSP 中的 startup_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/RTE/Device/ARMCM3/system_ARMCM3.c` | CMSIS · DSP 中的 system_ARMCM3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/RTE/Device/ARMCM4_FP/startup_ARMCM4.c` | CMSIS · DSP 中的 startup_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/RTE/Device/ARMCM4_FP/system_ARMCM4.c` | CMSIS · DSP 中的 system_ARMCM4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/RTE/Device/ARMCM55/startup_ARMCM55.c` | CMSIS · DSP 中的 startup_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/RTE/Device/ARMCM55/system_ARMCM55.c` | CMSIS · DSP 中的 system_ARMCM55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.c` | CMSIS · DSP 中的 startup_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c` | CMSIS · DSP 中的 system_ARMCM7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/arm_common_tables_f16.h` | CMSIS · DSP 中的 arm_common_tables_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/arm_common_tables.h` | CMSIS · DSP 中的 arm_common_tables 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/arm_const_structs_f16.h` | CMSIS · DSP 中的 arm_const_structs_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/arm_const_structs.h` | CMSIS · DSP 中的 arm_const_structs 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/arm_helium_utils.h` | CMSIS · DSP 中的 arm_helium_utils 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/arm_math_f16.h` | CMSIS · DSP 中的 arm_math_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/arm_math_memory.h` | CMSIS · DSP 中的 arm_math_memory 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/arm_math_types_f16.h` | CMSIS · DSP 中的 arm_math_types_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/arm_math_types.h` | CMSIS · DSP 中的 arm_math_types 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/arm_math.h` | CMSIS · DSP 中的 arm_math 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/arm_mve_tables_f16.h` | CMSIS · DSP 中的 arm_mve_tables_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/arm_mve_tables.h` | CMSIS · DSP 中的 arm_mve_tables 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/arm_vec_math_f16.h` | CMSIS · DSP 中的 arm_vec_math_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/arm_vec_math.h` | CMSIS · DSP 中的 arm_vec_math 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/basic_math_functions_f16.h` | CMSIS · DSP 中的 basic_math_functions_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/basic_math_functions.h` | CMSIS · DSP 中的 basic_math_functions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/bayes_functions_f16.h` | CMSIS · DSP 中的 bayes_functions_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/bayes_functions.h` | CMSIS · DSP 中的 bayes_functions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/complex_math_functions_f16.h` | CMSIS · DSP 中的 complex_math_functions_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/complex_math_functions.h` | CMSIS · DSP 中的 complex_math_functions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/controller_functions_f16.h` | CMSIS · DSP 中的 controller_functions_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/controller_functions.h` | CMSIS · DSP 中的 controller_functions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/distance_functions_f16.h` | CMSIS · DSP 中的 distance_functions_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/distance_functions.h` | CMSIS · DSP 中的 distance_functions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/fast_math_functions_f16.h` | CMSIS · DSP 中的 fast_math_functions_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/fast_math_functions.h` | CMSIS · DSP 中的 fast_math_functions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/filtering_functions_f16.h` | CMSIS · DSP 中的 filtering_functions_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/filtering_functions.h` | CMSIS · DSP 中的 filtering_functions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/interpolation_functions_f16.h` | CMSIS · DSP 中的 interpolation_functions_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/interpolation_functions.h` | CMSIS · DSP 中的 interpolation_functions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/matrix_functions_f16.h` | CMSIS · DSP 中的 matrix_functions_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/matrix_functions.h` | CMSIS · DSP 中的 matrix_functions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/none.h` | CMSIS · DSP 中的 none 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/quaternion_math_functions.h` | CMSIS · DSP 中的 quaternion_math_functions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/statistics_functions_f16.h` | CMSIS · DSP 中的 statistics_functions_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/statistics_functions.h` | CMSIS · DSP 中的 statistics_functions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/support_functions_f16.h` | CMSIS · DSP 中的 support_functions_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/support_functions.h` | CMSIS · DSP 中的 support_functions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/svm_defines.h` | CMSIS · DSP 中的 svm_defines 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/svm_functions_f16.h` | CMSIS · DSP 中的 svm_functions_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/svm_functions.h` | CMSIS · DSP 中的 svm_functions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/transform_functions_f16.h` | CMSIS · DSP 中的 transform_functions_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/transform_functions.h` | CMSIS · DSP 中的 transform_functions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Include/dsp/utils.h` | CMSIS · DSP 中的 utils 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/PrivateInclude/arm_sorting.h` | CMSIS · DSP 中的 arm_sorting 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/PrivateInclude/arm_vec_fft.h` | CMSIS · DSP 中的 arm_vec_fft 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/PrivateInclude/arm_vec_filtering.h` | CMSIS · DSP 中的 arm_vec_filtering 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_abs_f16.c` | CMSIS · DSP 中的 arm_abs_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_abs_f32.c` | CMSIS · DSP 中的 arm_abs_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_abs_f64.c` | CMSIS · DSP 中的 arm_abs_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_abs_q15.c` | CMSIS · DSP 中的 arm_abs_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_abs_q31.c` | CMSIS · DSP 中的 arm_abs_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_abs_q7.c` | CMSIS · DSP 中的 arm_abs_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_add_f16.c` | CMSIS · DSP 中的 arm_add_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_add_f32.c` | CMSIS · DSP 中的 arm_add_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_add_f64.c` | CMSIS · DSP 中的 arm_add_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_add_q15.c` | CMSIS · DSP 中的 arm_add_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_add_q31.c` | CMSIS · DSP 中的 arm_add_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_add_q7.c` | CMSIS · DSP 中的 arm_add_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_and_u16.c` | CMSIS · DSP 中的 arm_and_u16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_and_u32.c` | CMSIS · DSP 中的 arm_and_u32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_and_u8.c` | CMSIS · DSP 中的 arm_and_u8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_clip_f16.c` | CMSIS · DSP 中的 arm_clip_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_clip_f32.c` | CMSIS · DSP 中的 arm_clip_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_clip_q15.c` | CMSIS · DSP 中的 arm_clip_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_clip_q31.c` | CMSIS · DSP 中的 arm_clip_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_clip_q7.c` | CMSIS · DSP 中的 arm_clip_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_dot_prod_f16.c` | CMSIS · DSP 中的 arm_dot_prod_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_dot_prod_f32.c` | CMSIS · DSP 中的 arm_dot_prod_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_dot_prod_f64.c` | CMSIS · DSP 中的 arm_dot_prod_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_dot_prod_q15.c` | CMSIS · DSP 中的 arm_dot_prod_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_dot_prod_q31.c` | CMSIS · DSP 中的 arm_dot_prod_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_dot_prod_q7.c` | CMSIS · DSP 中的 arm_dot_prod_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_mult_f16.c` | CMSIS · DSP 中的 arm_mult_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_mult_f32.c` | CMSIS · DSP 中的 arm_mult_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_mult_f64.c` | CMSIS · DSP 中的 arm_mult_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_mult_q15.c` | CMSIS · DSP 中的 arm_mult_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_mult_q31.c` | CMSIS · DSP 中的 arm_mult_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_mult_q7.c` | CMSIS · DSP 中的 arm_mult_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_negate_f16.c` | CMSIS · DSP 中的 arm_negate_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_negate_f32.c` | CMSIS · DSP 中的 arm_negate_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_negate_f64.c` | CMSIS · DSP 中的 arm_negate_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_negate_q15.c` | CMSIS · DSP 中的 arm_negate_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_negate_q31.c` | CMSIS · DSP 中的 arm_negate_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_negate_q7.c` | CMSIS · DSP 中的 arm_negate_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_not_u16.c` | CMSIS · DSP 中的 arm_not_u16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_not_u32.c` | CMSIS · DSP 中的 arm_not_u32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_not_u8.c` | CMSIS · DSP 中的 arm_not_u8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_offset_f16.c` | CMSIS · DSP 中的 arm_offset_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_offset_f32.c` | CMSIS · DSP 中的 arm_offset_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_offset_f64.c` | CMSIS · DSP 中的 arm_offset_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_offset_q15.c` | CMSIS · DSP 中的 arm_offset_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_offset_q31.c` | CMSIS · DSP 中的 arm_offset_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_offset_q7.c` | CMSIS · DSP 中的 arm_offset_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_or_u16.c` | CMSIS · DSP 中的 arm_or_u16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_or_u32.c` | CMSIS · DSP 中的 arm_or_u32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_or_u8.c` | CMSIS · DSP 中的 arm_or_u8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_scale_f16.c` | CMSIS · DSP 中的 arm_scale_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_scale_f32.c` | CMSIS · DSP 中的 arm_scale_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_scale_f64.c` | CMSIS · DSP 中的 arm_scale_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_scale_q15.c` | CMSIS · DSP 中的 arm_scale_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_scale_q31.c` | CMSIS · DSP 中的 arm_scale_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_scale_q7.c` | CMSIS · DSP 中的 arm_scale_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_shift_q15.c` | CMSIS · DSP 中的 arm_shift_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_shift_q31.c` | CMSIS · DSP 中的 arm_shift_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_shift_q7.c` | CMSIS · DSP 中的 arm_shift_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_sub_f16.c` | CMSIS · DSP 中的 arm_sub_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_sub_f32.c` | CMSIS · DSP 中的 arm_sub_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_sub_f64.c` | CMSIS · DSP 中的 arm_sub_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_sub_q15.c` | CMSIS · DSP 中的 arm_sub_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_sub_q31.c` | CMSIS · DSP 中的 arm_sub_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_sub_q7.c` | CMSIS · DSP 中的 arm_sub_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_xor_u16.c` | CMSIS · DSP 中的 arm_xor_u16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_xor_u32.c` | CMSIS · DSP 中的 arm_xor_u32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_xor_u8.c` | CMSIS · DSP 中的 arm_xor_u8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/BasicMathFunctions.c` | CMSIS · DSP 中的 BasicMathFunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BasicMathFunctions/BasicMathFunctionsF16.c` | CMSIS · DSP 中的 BasicMathFunctionsF16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.c` | CMSIS · DSP 中的 arm_gaussian_naive_bayes_predict_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.c` | CMSIS · DSP 中的 arm_gaussian_naive_bayes_predict_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BayesFunctions/BayesFunctions.c` | CMSIS · DSP 中的 BayesFunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/BayesFunctions/BayesFunctionsF16.c` | CMSIS · DSP 中的 BayesFunctionsF16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables_f16.c` | CMSIS · DSP 中的 arm_common_tables_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c` | CMSIS · DSP 中的 arm_common_tables 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs_f16.c` | CMSIS · DSP 中的 arm_const_structs_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.c` | CMSIS · DSP 中的 arm_const_structs 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables_f16.c` | CMSIS · DSP 中的 arm_mve_tables_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables.c` | CMSIS · DSP 中的 arm_mve_tables 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/CommonTables/CommonTables.c` | CMSIS · DSP 中的 CommonTables 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/CommonTables/CommonTablesF16.c` | CMSIS · DSP 中的 CommonTablesF16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_conj_f16.c` | CMSIS · DSP 中的 arm_cmplx_conj_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_conj_f32.c` | CMSIS · DSP 中的 arm_cmplx_conj_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_conj_q15.c` | CMSIS · DSP 中的 arm_cmplx_conj_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_conj_q31.c` | CMSIS · DSP 中的 arm_cmplx_conj_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_dot_prod_f16.c` | CMSIS · DSP 中的 arm_cmplx_dot_prod_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_dot_prod_f32.c` | CMSIS · DSP 中的 arm_cmplx_dot_prod_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q15.c` | CMSIS · DSP 中的 arm_cmplx_dot_prod_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q31.c` | CMSIS · DSP 中的 arm_cmplx_dot_prod_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_f16.c` | CMSIS · DSP 中的 arm_cmplx_mag_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_f32.c` | CMSIS · DSP 中的 arm_cmplx_mag_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_f64.c` | CMSIS · DSP 中的 arm_cmplx_mag_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_fast_q15.c` | CMSIS · DSP 中的 arm_cmplx_mag_fast_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_q15.c` | CMSIS · DSP 中的 arm_cmplx_mag_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_q31.c` | CMSIS · DSP 中的 arm_cmplx_mag_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_squared_f16.c` | CMSIS · DSP 中的 arm_cmplx_mag_squared_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_squared_f32.c` | CMSIS · DSP 中的 arm_cmplx_mag_squared_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_squared_f64.c` | CMSIS · DSP 中的 arm_cmplx_mag_squared_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q15.c` | CMSIS · DSP 中的 arm_cmplx_mag_squared_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q31.c` | CMSIS · DSP 中的 arm_cmplx_mag_squared_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_f16.c` | CMSIS · DSP 中的 arm_cmplx_mult_cmplx_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.c` | CMSIS · DSP 中的 arm_cmplx_mult_cmplx_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_f64.c` | CMSIS · DSP 中的 arm_cmplx_mult_cmplx_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.c` | CMSIS · DSP 中的 arm_cmplx_mult_cmplx_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.c` | CMSIS · DSP 中的 arm_cmplx_mult_cmplx_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mult_real_f16.c` | CMSIS · DSP 中的 arm_cmplx_mult_real_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mult_real_f32.c` | CMSIS · DSP 中的 arm_cmplx_mult_real_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mult_real_q15.c` | CMSIS · DSP 中的 arm_cmplx_mult_real_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mult_real_q31.c` | CMSIS · DSP 中的 arm_cmplx_mult_real_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/ComplexMathFunctions.c` | CMSIS · DSP 中的 ComplexMathFunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ComplexMathFunctions/ComplexMathFunctionsF16.c` | CMSIS · DSP 中的 ComplexMathFunctionsF16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ControllerFunctions/arm_pid_init_f32.c` | CMSIS · DSP 中的 arm_pid_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ControllerFunctions/arm_pid_init_q15.c` | CMSIS · DSP 中的 arm_pid_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ControllerFunctions/arm_pid_init_q31.c` | CMSIS · DSP 中的 arm_pid_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ControllerFunctions/arm_pid_reset_f32.c` | CMSIS · DSP 中的 arm_pid_reset_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ControllerFunctions/arm_pid_reset_q15.c` | CMSIS · DSP 中的 arm_pid_reset_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ControllerFunctions/arm_pid_reset_q31.c` | CMSIS · DSP 中的 arm_pid_reset_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ControllerFunctions/arm_sin_cos_f32.c` | CMSIS · DSP 中的 arm_sin_cos_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ControllerFunctions/arm_sin_cos_q31.c` | CMSIS · DSP 中的 arm_sin_cos_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/ControllerFunctions/ControllerFunctions.c` | CMSIS · DSP 中的 ControllerFunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_boolean_distance_template.h` | CMSIS · DSP 中的 arm_boolean_distance_template 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_boolean_distance.c` | CMSIS · DSP 中的 arm_boolean_distance 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_braycurtis_distance_f16.c` | CMSIS · DSP 中的 arm_braycurtis_distance_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_braycurtis_distance_f32.c` | CMSIS · DSP 中的 arm_braycurtis_distance_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_canberra_distance_f16.c` | CMSIS · DSP 中的 arm_canberra_distance_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_canberra_distance_f32.c` | CMSIS · DSP 中的 arm_canberra_distance_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_chebyshev_distance_f16.c` | CMSIS · DSP 中的 arm_chebyshev_distance_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_chebyshev_distance_f32.c` | CMSIS · DSP 中的 arm_chebyshev_distance_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_chebyshev_distance_f64.c` | CMSIS · DSP 中的 arm_chebyshev_distance_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_cityblock_distance_f16.c` | CMSIS · DSP 中的 arm_cityblock_distance_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_cityblock_distance_f32.c` | CMSIS · DSP 中的 arm_cityblock_distance_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_cityblock_distance_f64.c` | CMSIS · DSP 中的 arm_cityblock_distance_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_correlation_distance_f16.c` | CMSIS · DSP 中的 arm_correlation_distance_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_correlation_distance_f32.c` | CMSIS · DSP 中的 arm_correlation_distance_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_cosine_distance_f16.c` | CMSIS · DSP 中的 arm_cosine_distance_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_cosine_distance_f32.c` | CMSIS · DSP 中的 arm_cosine_distance_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_cosine_distance_f64.c` | CMSIS · DSP 中的 arm_cosine_distance_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_dice_distance.c` | CMSIS · DSP 中的 arm_dice_distance 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_euclidean_distance_f16.c` | CMSIS · DSP 中的 arm_euclidean_distance_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_euclidean_distance_f32.c` | CMSIS · DSP 中的 arm_euclidean_distance_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_euclidean_distance_f64.c` | CMSIS · DSP 中的 arm_euclidean_distance_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_hamming_distance.c` | CMSIS · DSP 中的 arm_hamming_distance 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_jaccard_distance.c` | CMSIS · DSP 中的 arm_jaccard_distance 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_jensenshannon_distance_f16.c` | CMSIS · DSP 中的 arm_jensenshannon_distance_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_jensenshannon_distance_f32.c` | CMSIS · DSP 中的 arm_jensenshannon_distance_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_kulsinski_distance.c` | CMSIS · DSP 中的 arm_kulsinski_distance 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_minkowski_distance_f16.c` | CMSIS · DSP 中的 arm_minkowski_distance_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_minkowski_distance_f32.c` | CMSIS · DSP 中的 arm_minkowski_distance_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_rogerstanimoto_distance.c` | CMSIS · DSP 中的 arm_rogerstanimoto_distance 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_russellrao_distance.c` | CMSIS · DSP 中的 arm_russellrao_distance 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_sokalmichener_distance.c` | CMSIS · DSP 中的 arm_sokalmichener_distance 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_sokalsneath_distance.c` | CMSIS · DSP 中的 arm_sokalsneath_distance 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/arm_yule_distance.c` | CMSIS · DSP 中的 arm_yule_distance 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/DistanceFunctions.c` | CMSIS · DSP 中的 DistanceFunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/DistanceFunctions/DistanceFunctionsF16.c` | CMSIS · DSP 中的 DistanceFunctionsF16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_atan2_f16.c` | CMSIS · DSP 中的 arm_atan2_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_atan2_f32.c` | CMSIS · DSP 中的 arm_atan2_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_atan2_q15.c` | CMSIS · DSP 中的 arm_atan2_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_atan2_q31.c` | CMSIS · DSP 中的 arm_atan2_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_cos_f32.c` | CMSIS · DSP 中的 arm_cos_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_cos_q15.c` | CMSIS · DSP 中的 arm_cos_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_cos_q31.c` | CMSIS · DSP 中的 arm_cos_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_divide_q15.c` | CMSIS · DSP 中的 arm_divide_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_divide_q31.c` | CMSIS · DSP 中的 arm_divide_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c` | CMSIS · DSP 中的 arm_sin_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_q15.c` | CMSIS · DSP 中的 arm_sin_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_q31.c` | CMSIS · DSP 中的 arm_sin_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sqrt_q15.c` | CMSIS · DSP 中的 arm_sqrt_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sqrt_q31.c` | CMSIS · DSP 中的 arm_sqrt_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_vexp_f16.c` | CMSIS · DSP 中的 arm_vexp_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_vexp_f32.c` | CMSIS · DSP 中的 arm_vexp_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_vexp_f64.c` | CMSIS · DSP 中的 arm_vexp_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_vinverse_f16.c` | CMSIS · DSP 中的 arm_vinverse_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_vlog_f16.c` | CMSIS · DSP 中的 arm_vlog_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_vlog_f32.c` | CMSIS · DSP 中的 arm_vlog_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_vlog_f64.c` | CMSIS · DSP 中的 arm_vlog_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_vlog_q15.c` | CMSIS · DSP 中的 arm_vlog_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_vlog_q31.c` | CMSIS · DSP 中的 arm_vlog_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/FastMathFunctions.c` | CMSIS · DSP 中的 FastMathFunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FastMathFunctions/FastMathFunctionsF16.c` | CMSIS · DSP 中的 FastMathFunctionsF16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_32x64_init_q31.c` | CMSIS · DSP 中的 arm_biquad_cascade_df1_32x64_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_32x64_q31.c` | CMSIS · DSP 中的 arm_biquad_cascade_df1_32x64_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_f16.c` | CMSIS · DSP 中的 arm_biquad_cascade_df1_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_f32.c` | CMSIS · DSP 中的 arm_biquad_cascade_df1_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_fast_q15.c` | CMSIS · DSP 中的 arm_biquad_cascade_df1_fast_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_fast_q31.c` | CMSIS · DSP 中的 arm_biquad_cascade_df1_fast_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_init_f16.c` | CMSIS · DSP 中的 arm_biquad_cascade_df1_init_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c` | CMSIS · DSP 中的 arm_biquad_cascade_df1_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_init_q15.c` | CMSIS · DSP 中的 arm_biquad_cascade_df1_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_init_q31.c` | CMSIS · DSP 中的 arm_biquad_cascade_df1_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_q15.c` | CMSIS · DSP 中的 arm_biquad_cascade_df1_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_q31.c` | CMSIS · DSP 中的 arm_biquad_cascade_df1_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df2T_f16.c` | CMSIS · DSP 中的 arm_biquad_cascade_df2T_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df2T_f32.c` | CMSIS · DSP 中的 arm_biquad_cascade_df2T_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df2T_f64.c` | CMSIS · DSP 中的 arm_biquad_cascade_df2T_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df2T_init_f16.c` | CMSIS · DSP 中的 arm_biquad_cascade_df2T_init_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df2T_init_f32.c` | CMSIS · DSP 中的 arm_biquad_cascade_df2T_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df2T_init_f64.c` | CMSIS · DSP 中的 arm_biquad_cascade_df2T_init_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_stereo_df2T_f16.c` | CMSIS · DSP 中的 arm_biquad_cascade_stereo_df2T_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_stereo_df2T_f32.c` | CMSIS · DSP 中的 arm_biquad_cascade_stereo_df2T_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_stereo_df2T_init_f16.c` | CMSIS · DSP 中的 arm_biquad_cascade_stereo_df2T_init_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_stereo_df2T_init_f32.c` | CMSIS · DSP 中的 arm_biquad_cascade_stereo_df2T_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_f32.c` | CMSIS · DSP 中的 arm_conv_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_fast_opt_q15.c` | CMSIS · DSP 中的 arm_conv_fast_opt_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_fast_q15.c` | CMSIS · DSP 中的 arm_conv_fast_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_fast_q31.c` | CMSIS · DSP 中的 arm_conv_fast_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_opt_q15.c` | CMSIS · DSP 中的 arm_conv_opt_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_opt_q7.c` | CMSIS · DSP 中的 arm_conv_opt_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_f32.c` | CMSIS · DSP 中的 arm_conv_partial_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_fast_opt_q15.c` | CMSIS · DSP 中的 arm_conv_partial_fast_opt_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_fast_q15.c` | CMSIS · DSP 中的 arm_conv_partial_fast_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_fast_q31.c` | CMSIS · DSP 中的 arm_conv_partial_fast_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_opt_q15.c` | CMSIS · DSP 中的 arm_conv_partial_opt_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_opt_q7.c` | CMSIS · DSP 中的 arm_conv_partial_opt_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_q15.c` | CMSIS · DSP 中的 arm_conv_partial_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_q31.c` | CMSIS · DSP 中的 arm_conv_partial_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_q7.c` | CMSIS · DSP 中的 arm_conv_partial_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_q15.c` | CMSIS · DSP 中的 arm_conv_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_q31.c` | CMSIS · DSP 中的 arm_conv_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_conv_q7.c` | CMSIS · DSP 中的 arm_conv_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_f16.c` | CMSIS · DSP 中的 arm_correlate_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_f32.c` | CMSIS · DSP 中的 arm_correlate_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_f64.c` | CMSIS · DSP 中的 arm_correlate_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_fast_opt_q15.c` | CMSIS · DSP 中的 arm_correlate_fast_opt_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_fast_q15.c` | CMSIS · DSP 中的 arm_correlate_fast_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_fast_q31.c` | CMSIS · DSP 中的 arm_correlate_fast_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_opt_q15.c` | CMSIS · DSP 中的 arm_correlate_opt_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_opt_q7.c` | CMSIS · DSP 中的 arm_correlate_opt_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_q15.c` | CMSIS · DSP 中的 arm_correlate_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_q31.c` | CMSIS · DSP 中的 arm_correlate_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_q7.c` | CMSIS · DSP 中的 arm_correlate_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_f32.c` | CMSIS · DSP 中的 arm_fir_decimate_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_fast_q15.c` | CMSIS · DSP 中的 arm_fir_decimate_fast_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_fast_q31.c` | CMSIS · DSP 中的 arm_fir_decimate_fast_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_init_f32.c` | CMSIS · DSP 中的 arm_fir_decimate_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_init_q15.c` | CMSIS · DSP 中的 arm_fir_decimate_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_init_q31.c` | CMSIS · DSP 中的 arm_fir_decimate_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_q15.c` | CMSIS · DSP 中的 arm_fir_decimate_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_q31.c` | CMSIS · DSP 中的 arm_fir_decimate_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_f16.c` | CMSIS · DSP 中的 arm_fir_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_f32.c` | CMSIS · DSP 中的 arm_fir_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_f64.c` | CMSIS · DSP 中的 arm_fir_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_fast_q15.c` | CMSIS · DSP 中的 arm_fir_fast_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_fast_q31.c` | CMSIS · DSP 中的 arm_fir_fast_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_init_f16.c` | CMSIS · DSP 中的 arm_fir_init_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_init_f32.c` | CMSIS · DSP 中的 arm_fir_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_init_f64.c` | CMSIS · DSP 中的 arm_fir_init_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_init_q15.c` | CMSIS · DSP 中的 arm_fir_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_init_q31.c` | CMSIS · DSP 中的 arm_fir_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_init_q7.c` | CMSIS · DSP 中的 arm_fir_init_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_f32.c` | CMSIS · DSP 中的 arm_fir_interpolate_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_init_f32.c` | CMSIS · DSP 中的 arm_fir_interpolate_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_init_q15.c` | CMSIS · DSP 中的 arm_fir_interpolate_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_init_q31.c` | CMSIS · DSP 中的 arm_fir_interpolate_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_q15.c` | CMSIS · DSP 中的 arm_fir_interpolate_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_q31.c` | CMSIS · DSP 中的 arm_fir_interpolate_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_lattice_f32.c` | CMSIS · DSP 中的 arm_fir_lattice_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_lattice_init_f32.c` | CMSIS · DSP 中的 arm_fir_lattice_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_lattice_init_q15.c` | CMSIS · DSP 中的 arm_fir_lattice_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_lattice_init_q31.c` | CMSIS · DSP 中的 arm_fir_lattice_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_lattice_q15.c` | CMSIS · DSP 中的 arm_fir_lattice_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_lattice_q31.c` | CMSIS · DSP 中的 arm_fir_lattice_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_q15.c` | CMSIS · DSP 中的 arm_fir_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_q31.c` | CMSIS · DSP 中的 arm_fir_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_q7.c` | CMSIS · DSP 中的 arm_fir_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_f32.c` | CMSIS · DSP 中的 arm_fir_sparse_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_init_f32.c` | CMSIS · DSP 中的 arm_fir_sparse_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_init_q15.c` | CMSIS · DSP 中的 arm_fir_sparse_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_init_q31.c` | CMSIS · DSP 中的 arm_fir_sparse_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_init_q7.c` | CMSIS · DSP 中的 arm_fir_sparse_init_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_q15.c` | CMSIS · DSP 中的 arm_fir_sparse_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_q31.c` | CMSIS · DSP 中的 arm_fir_sparse_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_q7.c` | CMSIS · DSP 中的 arm_fir_sparse_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_iir_lattice_f32.c` | CMSIS · DSP 中的 arm_iir_lattice_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_iir_lattice_init_f32.c` | CMSIS · DSP 中的 arm_iir_lattice_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_iir_lattice_init_q15.c` | CMSIS · DSP 中的 arm_iir_lattice_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_iir_lattice_init_q31.c` | CMSIS · DSP 中的 arm_iir_lattice_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_iir_lattice_q15.c` | CMSIS · DSP 中的 arm_iir_lattice_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_iir_lattice_q31.c` | CMSIS · DSP 中的 arm_iir_lattice_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_levinson_durbin_f16.c` | CMSIS · DSP 中的 arm_levinson_durbin_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_levinson_durbin_f32.c` | CMSIS · DSP 中的 arm_levinson_durbin_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_levinson_durbin_q31.c` | CMSIS · DSP 中的 arm_levinson_durbin_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_lms_f32.c` | CMSIS · DSP 中的 arm_lms_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_lms_init_f32.c` | CMSIS · DSP 中的 arm_lms_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_lms_init_q15.c` | CMSIS · DSP 中的 arm_lms_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_lms_init_q31.c` | CMSIS · DSP 中的 arm_lms_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_lms_norm_f32.c` | CMSIS · DSP 中的 arm_lms_norm_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_lms_norm_init_f32.c` | CMSIS · DSP 中的 arm_lms_norm_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_lms_norm_init_q15.c` | CMSIS · DSP 中的 arm_lms_norm_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_lms_norm_init_q31.c` | CMSIS · DSP 中的 arm_lms_norm_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_lms_norm_q15.c` | CMSIS · DSP 中的 arm_lms_norm_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_lms_norm_q31.c` | CMSIS · DSP 中的 arm_lms_norm_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_lms_q15.c` | CMSIS · DSP 中的 arm_lms_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_lms_q31.c` | CMSIS · DSP 中的 arm_lms_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/FilteringFunctions.c` | CMSIS · DSP 中的 FilteringFunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/FilteringFunctions/FilteringFunctionsF16.c` | CMSIS · DSP 中的 FilteringFunctionsF16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_f16.c` | CMSIS · DSP 中的 arm_bilinear_interp_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_f32.c` | CMSIS · DSP 中的 arm_bilinear_interp_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q15.c` | CMSIS · DSP 中的 arm_bilinear_interp_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q31.c` | CMSIS · DSP 中的 arm_bilinear_interp_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/InterpolationFunctions/arm_bilinear_interp_q7.c` | CMSIS · DSP 中的 arm_bilinear_interp_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_f16.c` | CMSIS · DSP 中的 arm_linear_interp_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_f32.c` | CMSIS · DSP 中的 arm_linear_interp_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q15.c` | CMSIS · DSP 中的 arm_linear_interp_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q31.c` | CMSIS · DSP 中的 arm_linear_interp_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/InterpolationFunctions/arm_linear_interp_q7.c` | CMSIS · DSP 中的 arm_linear_interp_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/InterpolationFunctions/arm_spline_interp_f32.c` | CMSIS · DSP 中的 arm_spline_interp_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/InterpolationFunctions/arm_spline_interp_init_f32.c` | CMSIS · DSP 中的 arm_spline_interp_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.c` | CMSIS · DSP 中的 InterpolationFunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctionsF16.c` | CMSIS · DSP 中的 InterpolationFunctionsF16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f16.c` | CMSIS · DSP 中的 arm_mat_add_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.c` | CMSIS · DSP 中的 arm_mat_add_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q15.c` | CMSIS · DSP 中的 arm_mat_add_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q31.c` | CMSIS · DSP 中的 arm_mat_add_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.c` | CMSIS · DSP 中的 arm_mat_cholesky_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.c` | CMSIS · DSP 中的 arm_mat_cholesky_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.c` | CMSIS · DSP 中的 arm_mat_cholesky_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.c` | CMSIS · DSP 中的 arm_mat_cmplx_mult_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.c` | CMSIS · DSP 中的 arm_mat_cmplx_mult_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.c` | CMSIS · DSP 中的 arm_mat_cmplx_mult_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.c` | CMSIS · DSP 中的 arm_mat_cmplx_mult_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.c` | CMSIS · DSP 中的 arm_mat_cmplx_trans_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.c` | CMSIS · DSP 中的 arm_mat_cmplx_trans_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.c` | CMSIS · DSP 中的 arm_mat_cmplx_trans_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.c` | CMSIS · DSP 中的 arm_mat_cmplx_trans_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f16.c` | CMSIS · DSP 中的 arm_mat_init_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.c` | CMSIS · DSP 中的 arm_mat_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q15.c` | CMSIS · DSP 中的 arm_mat_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q31.c` | CMSIS · DSP 中的 arm_mat_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f16.c` | CMSIS · DSP 中的 arm_mat_inverse_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c` | CMSIS · DSP 中的 arm_mat_inverse_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f64.c` | CMSIS · DSP 中的 arm_mat_inverse_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.c` | CMSIS · DSP 中的 arm_mat_ldlt_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.c` | CMSIS · DSP 中的 arm_mat_ldlt_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f16.c` | CMSIS · DSP 中的 arm_mat_mult_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c` | CMSIS · DSP 中的 arm_mat_mult_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f64.c` | CMSIS · DSP 中的 arm_mat_mult_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.c` | CMSIS · DSP 中的 arm_mat_mult_fast_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.c` | CMSIS · DSP 中的 arm_mat_mult_fast_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_opt_q31.c` | CMSIS · DSP 中的 arm_mat_mult_opt_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q15.c` | CMSIS · DSP 中的 arm_mat_mult_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q31.c` | CMSIS · DSP 中的 arm_mat_mult_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q7.c` | CMSIS · DSP 中的 arm_mat_mult_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f16.c` | CMSIS · DSP 中的 arm_mat_scale_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f32.c` | CMSIS · DSP 中的 arm_mat_scale_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q15.c` | CMSIS · DSP 中的 arm_mat_scale_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q31.c` | CMSIS · DSP 中的 arm_mat_scale_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.c` | CMSIS · DSP 中的 arm_mat_solve_lower_triangular_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.c` | CMSIS · DSP 中的 arm_mat_solve_lower_triangular_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.c` | CMSIS · DSP 中的 arm_mat_solve_lower_triangular_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.c` | CMSIS · DSP 中的 arm_mat_solve_upper_triangular_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.c` | CMSIS · DSP 中的 arm_mat_solve_upper_triangular_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.c` | CMSIS · DSP 中的 arm_mat_solve_upper_triangular_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f16.c` | CMSIS · DSP 中的 arm_mat_sub_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c` | CMSIS · DSP 中的 arm_mat_sub_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f64.c` | CMSIS · DSP 中的 arm_mat_sub_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q15.c` | CMSIS · DSP 中的 arm_mat_sub_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q31.c` | CMSIS · DSP 中的 arm_mat_sub_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f16.c` | CMSIS · DSP 中的 arm_mat_trans_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c` | CMSIS · DSP 中的 arm_mat_trans_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f64.c` | CMSIS · DSP 中的 arm_mat_trans_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q15.c` | CMSIS · DSP 中的 arm_mat_trans_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q31.c` | CMSIS · DSP 中的 arm_mat_trans_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q7.c` | CMSIS · DSP 中的 arm_mat_trans_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.c` | CMSIS · DSP 中的 arm_mat_vec_mult_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.c` | CMSIS · DSP 中的 arm_mat_vec_mult_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.c` | CMSIS · DSP 中的 arm_mat_vec_mult_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.c` | CMSIS · DSP 中的 arm_mat_vec_mult_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.c` | CMSIS · DSP 中的 arm_mat_vec_mult_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctions.c` | CMSIS · DSP 中的 MatrixFunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctionsF16.c` | CMSIS · DSP 中的 MatrixFunctionsF16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.c` | CMSIS · DSP 中的 arm_quaternion_conjugate_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.c` | CMSIS · DSP 中的 arm_quaternion_inverse_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.c` | CMSIS · DSP 中的 arm_quaternion_norm_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.c` | CMSIS · DSP 中的 arm_quaternion_normalize_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/QuaternionMathFunctions/arm_quaternion_product_f32.c` | CMSIS · DSP 中的 arm_quaternion_product_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.c` | CMSIS · DSP 中的 arm_quaternion_product_single_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.c` | CMSIS · DSP 中的 arm_quaternion2rotation_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.c` | CMSIS · DSP 中的 arm_rotation2quaternion_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/QuaternionMathFunctions/QuaternionMathFunctions.c` | CMSIS · DSP 中的 QuaternionMathFunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmax_f16.c` | CMSIS · DSP 中的 arm_absmax_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmax_f32.c` | CMSIS · DSP 中的 arm_absmax_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmax_f64.c` | CMSIS · DSP 中的 arm_absmax_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmax_no_idx_f16.c` | CMSIS · DSP 中的 arm_absmax_no_idx_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmax_no_idx_f32.c` | CMSIS · DSP 中的 arm_absmax_no_idx_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmax_no_idx_f64.c` | CMSIS · DSP 中的 arm_absmax_no_idx_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmax_no_idx_q15.c` | CMSIS · DSP 中的 arm_absmax_no_idx_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmax_no_idx_q31.c` | CMSIS · DSP 中的 arm_absmax_no_idx_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmax_no_idx_q7.c` | CMSIS · DSP 中的 arm_absmax_no_idx_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmax_q15.c` | CMSIS · DSP 中的 arm_absmax_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmax_q31.c` | CMSIS · DSP 中的 arm_absmax_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmax_q7.c` | CMSIS · DSP 中的 arm_absmax_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmin_f16.c` | CMSIS · DSP 中的 arm_absmin_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmin_f32.c` | CMSIS · DSP 中的 arm_absmin_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmin_f64.c` | CMSIS · DSP 中的 arm_absmin_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmin_no_idx_f16.c` | CMSIS · DSP 中的 arm_absmin_no_idx_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmin_no_idx_f32.c` | CMSIS · DSP 中的 arm_absmin_no_idx_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmin_no_idx_f64.c` | CMSIS · DSP 中的 arm_absmin_no_idx_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmin_no_idx_q15.c` | CMSIS · DSP 中的 arm_absmin_no_idx_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmin_no_idx_q31.c` | CMSIS · DSP 中的 arm_absmin_no_idx_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmin_no_idx_q7.c` | CMSIS · DSP 中的 arm_absmin_no_idx_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmin_q15.c` | CMSIS · DSP 中的 arm_absmin_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmin_q31.c` | CMSIS · DSP 中的 arm_absmin_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_absmin_q7.c` | CMSIS · DSP 中的 arm_absmin_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_entropy_f16.c` | CMSIS · DSP 中的 arm_entropy_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_entropy_f32.c` | CMSIS · DSP 中的 arm_entropy_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_entropy_f64.c` | CMSIS · DSP 中的 arm_entropy_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_kullback_leibler_f16.c` | CMSIS · DSP 中的 arm_kullback_leibler_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_kullback_leibler_f32.c` | CMSIS · DSP 中的 arm_kullback_leibler_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_kullback_leibler_f64.c` | CMSIS · DSP 中的 arm_kullback_leibler_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f16.c` | CMSIS · DSP 中的 arm_logsumexp_dot_prod_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f32.c` | CMSIS · DSP 中的 arm_logsumexp_dot_prod_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_logsumexp_f16.c` | CMSIS · DSP 中的 arm_logsumexp_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_logsumexp_f32.c` | CMSIS · DSP 中的 arm_logsumexp_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_max_f16.c` | CMSIS · DSP 中的 arm_max_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_max_f32.c` | CMSIS · DSP 中的 arm_max_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_max_f64.c` | CMSIS · DSP 中的 arm_max_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_max_no_idx_f16.c` | CMSIS · DSP 中的 arm_max_no_idx_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_max_no_idx_f32.c` | CMSIS · DSP 中的 arm_max_no_idx_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_max_no_idx_f64.c` | CMSIS · DSP 中的 arm_max_no_idx_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_max_no_idx_q15.c` | CMSIS · DSP 中的 arm_max_no_idx_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_max_no_idx_q31.c` | CMSIS · DSP 中的 arm_max_no_idx_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_max_no_idx_q7.c` | CMSIS · DSP 中的 arm_max_no_idx_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_max_q15.c` | CMSIS · DSP 中的 arm_max_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_max_q31.c` | CMSIS · DSP 中的 arm_max_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_max_q7.c` | CMSIS · DSP 中的 arm_max_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_mean_f16.c` | CMSIS · DSP 中的 arm_mean_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_mean_f32.c` | CMSIS · DSP 中的 arm_mean_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_mean_f64.c` | CMSIS · DSP 中的 arm_mean_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_mean_q15.c` | CMSIS · DSP 中的 arm_mean_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_mean_q31.c` | CMSIS · DSP 中的 arm_mean_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_mean_q7.c` | CMSIS · DSP 中的 arm_mean_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_min_f16.c` | CMSIS · DSP 中的 arm_min_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_min_f32.c` | CMSIS · DSP 中的 arm_min_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_min_f64.c` | CMSIS · DSP 中的 arm_min_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_min_no_idx_f16.c` | CMSIS · DSP 中的 arm_min_no_idx_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_min_no_idx_f32.c` | CMSIS · DSP 中的 arm_min_no_idx_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_min_no_idx_f64.c` | CMSIS · DSP 中的 arm_min_no_idx_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_min_no_idx_q15.c` | CMSIS · DSP 中的 arm_min_no_idx_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_min_no_idx_q31.c` | CMSIS · DSP 中的 arm_min_no_idx_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_min_no_idx_q7.c` | CMSIS · DSP 中的 arm_min_no_idx_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_min_q15.c` | CMSIS · DSP 中的 arm_min_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_min_q31.c` | CMSIS · DSP 中的 arm_min_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_min_q7.c` | CMSIS · DSP 中的 arm_min_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_mse_f16.c` | CMSIS · DSP 中的 arm_mse_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_mse_f32.c` | CMSIS · DSP 中的 arm_mse_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_mse_f64.c` | CMSIS · DSP 中的 arm_mse_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_mse_q15.c` | CMSIS · DSP 中的 arm_mse_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_mse_q31.c` | CMSIS · DSP 中的 arm_mse_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_mse_q7.c` | CMSIS · DSP 中的 arm_mse_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_power_f16.c` | CMSIS · DSP 中的 arm_power_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_power_f32.c` | CMSIS · DSP 中的 arm_power_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_power_f64.c` | CMSIS · DSP 中的 arm_power_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_power_q15.c` | CMSIS · DSP 中的 arm_power_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_power_q31.c` | CMSIS · DSP 中的 arm_power_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_power_q7.c` | CMSIS · DSP 中的 arm_power_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_rms_f16.c` | CMSIS · DSP 中的 arm_rms_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_rms_f32.c` | CMSIS · DSP 中的 arm_rms_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_rms_q15.c` | CMSIS · DSP 中的 arm_rms_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_rms_q31.c` | CMSIS · DSP 中的 arm_rms_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_std_f16.c` | CMSIS · DSP 中的 arm_std_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_std_f32.c` | CMSIS · DSP 中的 arm_std_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_std_f64.c` | CMSIS · DSP 中的 arm_std_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_std_q15.c` | CMSIS · DSP 中的 arm_std_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_std_q31.c` | CMSIS · DSP 中的 arm_std_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_var_f16.c` | CMSIS · DSP 中的 arm_var_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_var_f32.c` | CMSIS · DSP 中的 arm_var_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_var_f64.c` | CMSIS · DSP 中的 arm_var_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_var_q15.c` | CMSIS · DSP 中的 arm_var_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_var_q31.c` | CMSIS · DSP 中的 arm_var_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/StatisticsFunctions.c` | CMSIS · DSP 中的 StatisticsFunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/StatisticsFunctions/StatisticsFunctionsF16.c` | CMSIS · DSP 中的 StatisticsFunctionsF16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_barycenter_f16.c` | CMSIS · DSP 中的 arm_barycenter_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_barycenter_f32.c` | CMSIS · DSP 中的 arm_barycenter_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_bitonic_sort_f32.c` | CMSIS · DSP 中的 arm_bitonic_sort_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_bubble_sort_f32.c` | CMSIS · DSP 中的 arm_bubble_sort_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f16.c` | CMSIS · DSP 中的 arm_copy_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f32.c` | CMSIS · DSP 中的 arm_copy_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f64.c` | CMSIS · DSP 中的 arm_copy_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.c` | CMSIS · DSP 中的 arm_copy_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q31.c` | CMSIS · DSP 中的 arm_copy_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.c` | CMSIS · DSP 中的 arm_copy_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_f16_to_float.c` | CMSIS · DSP 中的 arm_f16_to_float 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_f16_to_q15.c` | CMSIS · DSP 中的 arm_f16_to_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f16.c` | CMSIS · DSP 中的 arm_fill_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f32.c` | CMSIS · DSP 中的 arm_fill_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f64.c` | CMSIS · DSP 中的 arm_fill_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.c` | CMSIS · DSP 中的 arm_fill_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q31.c` | CMSIS · DSP 中的 arm_fill_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.c` | CMSIS · DSP 中的 arm_fill_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_f16.c` | CMSIS · DSP 中的 arm_float_to_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.c` | CMSIS · DSP 中的 arm_float_to_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.c` | CMSIS · DSP 中的 arm_float_to_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.c` | CMSIS · DSP 中的 arm_float_to_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_heap_sort_f32.c` | CMSIS · DSP 中的 arm_heap_sort_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_insertion_sort_f32.c` | CMSIS · DSP 中的 arm_insertion_sort_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_merge_sort_f32.c` | CMSIS · DSP 中的 arm_merge_sort_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_merge_sort_init_f32.c` | CMSIS · DSP 中的 arm_merge_sort_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_f16.c` | CMSIS · DSP 中的 arm_q15_to_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.c` | CMSIS · DSP 中的 arm_q15_to_float 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q31.c` | CMSIS · DSP 中的 arm_q15_to_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.c` | CMSIS · DSP 中的 arm_q15_to_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.c` | CMSIS · DSP 中的 arm_q31_to_float 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q15.c` | CMSIS · DSP 中的 arm_q31_to_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q7.c` | CMSIS · DSP 中的 arm_q31_to_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.c` | CMSIS · DSP 中的 arm_q7_to_float 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.c` | CMSIS · DSP 中的 arm_q7_to_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q31.c` | CMSIS · DSP 中的 arm_q7_to_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_quick_sort_f32.c` | CMSIS · DSP 中的 arm_quick_sort_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_selection_sort_f32.c` | CMSIS · DSP 中的 arm_selection_sort_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_sort_f32.c` | CMSIS · DSP 中的 arm_sort_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_sort_init_f32.c` | CMSIS · DSP 中的 arm_sort_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_weighted_sum_f16.c` | CMSIS · DSP 中的 arm_weighted_sum_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/arm_weighted_sum_f32.c` | CMSIS · DSP 中的 arm_weighted_sum_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/SupportFunctions.c` | CMSIS · DSP 中的 SupportFunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SupportFunctions/SupportFunctionsF16.c` | CMSIS · DSP 中的 SupportFunctionsF16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_linear_init_f16.c` | CMSIS · DSP 中的 arm_svm_linear_init_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_linear_init_f32.c` | CMSIS · DSP 中的 arm_svm_linear_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_linear_predict_f16.c` | CMSIS · DSP 中的 arm_svm_linear_predict_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_linear_predict_f32.c` | CMSIS · DSP 中的 arm_svm_linear_predict_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_polynomial_init_f16.c` | CMSIS · DSP 中的 arm_svm_polynomial_init_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_polynomial_init_f32.c` | CMSIS · DSP 中的 arm_svm_polynomial_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f16.c` | CMSIS · DSP 中的 arm_svm_polynomial_predict_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f32.c` | CMSIS · DSP 中的 arm_svm_polynomial_predict_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_rbf_init_f16.c` | CMSIS · DSP 中的 arm_svm_rbf_init_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_rbf_init_f32.c` | CMSIS · DSP 中的 arm_svm_rbf_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_rbf_predict_f16.c` | CMSIS · DSP 中的 arm_svm_rbf_predict_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_rbf_predict_f32.c` | CMSIS · DSP 中的 arm_svm_rbf_predict_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f16.c` | CMSIS · DSP 中的 arm_svm_sigmoid_init_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f32.c` | CMSIS · DSP 中的 arm_svm_sigmoid_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f16.c` | CMSIS · DSP 中的 arm_svm_sigmoid_predict_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f32.c` | CMSIS · DSP 中的 arm_svm_sigmoid_predict_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/SVMFunctions.c` | CMSIS · DSP 中的 SVMFunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/SVMFunctions/SVMFunctionsF16.c` | CMSIS · DSP 中的 SVMFunctionsF16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal_f16.c` | CMSIS · DSP 中的 arm_bitreversal_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal.c` | CMSIS · DSP 中的 arm_bitreversal 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal2.c` | CMSIS · DSP 中的 arm_bitreversal2 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_f16.c` | CMSIS · DSP 中的 arm_cfft_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_f32.c` | CMSIS · DSP 中的 arm_cfft_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_f64.c` | CMSIS · DSP 中的 arm_cfft_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_init_f16.c` | CMSIS · DSP 中的 arm_cfft_init_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_init_f32.c` | CMSIS · DSP 中的 arm_cfft_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_init_f64.c` | CMSIS · DSP 中的 arm_cfft_init_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_init_q15.c` | CMSIS · DSP 中的 arm_cfft_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_init_q31.c` | CMSIS · DSP 中的 arm_cfft_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_q15.c` | CMSIS · DSP 中的 arm_cfft_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_q31.c` | CMSIS · DSP 中的 arm_cfft_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix2_f16.c` | CMSIS · DSP 中的 arm_cfft_radix2_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix2_f32.c` | CMSIS · DSP 中的 arm_cfft_radix2_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix2_init_f16.c` | CMSIS · DSP 中的 arm_cfft_radix2_init_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix2_init_f32.c` | CMSIS · DSP 中的 arm_cfft_radix2_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix2_init_q15.c` | CMSIS · DSP 中的 arm_cfft_radix2_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix2_init_q31.c` | CMSIS · DSP 中的 arm_cfft_radix2_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix2_q15.c` | CMSIS · DSP 中的 arm_cfft_radix2_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix2_q31.c` | CMSIS · DSP 中的 arm_cfft_radix2_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_f16.c` | CMSIS · DSP 中的 arm_cfft_radix4_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_f32.c` | CMSIS · DSP 中的 arm_cfft_radix4_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_init_f16.c` | CMSIS · DSP 中的 arm_cfft_radix4_init_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_init_f32.c` | CMSIS · DSP 中的 arm_cfft_radix4_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_init_q15.c` | CMSIS · DSP 中的 arm_cfft_radix4_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_init_q31.c` | CMSIS · DSP 中的 arm_cfft_radix4_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_q15.c` | CMSIS · DSP 中的 arm_cfft_radix4_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_q31.c` | CMSIS · DSP 中的 arm_cfft_radix4_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix8_f16.c` | CMSIS · DSP 中的 arm_cfft_radix8_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix8_f32.c` | CMSIS · DSP 中的 arm_cfft_radix8_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_dct4_f32.c` | CMSIS · DSP 中的 arm_dct4_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_dct4_init_f32.c` | CMSIS · DSP 中的 arm_dct4_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_dct4_init_q15.c` | CMSIS · DSP 中的 arm_dct4_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_dct4_init_q31.c` | CMSIS · DSP 中的 arm_dct4_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_dct4_q15.c` | CMSIS · DSP 中的 arm_dct4_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_dct4_q31.c` | CMSIS · DSP 中的 arm_dct4_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_mfcc_f16.c` | CMSIS · DSP 中的 arm_mfcc_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_mfcc_f32.c` | CMSIS · DSP 中的 arm_mfcc_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_mfcc_init_f16.c` | CMSIS · DSP 中的 arm_mfcc_init_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_mfcc_init_f32.c` | CMSIS · DSP 中的 arm_mfcc_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_mfcc_init_q15.c` | CMSIS · DSP 中的 arm_mfcc_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_mfcc_init_q31.c` | CMSIS · DSP 中的 arm_mfcc_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_mfcc_q15.c` | CMSIS · DSP 中的 arm_mfcc_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_mfcc_q31.c` | CMSIS · DSP 中的 arm_mfcc_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_f32.c` | CMSIS · DSP 中的 arm_rfft_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_f16.c` | CMSIS · DSP 中的 arm_rfft_fast_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_f32.c` | CMSIS · DSP 中的 arm_rfft_fast_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_f64.c` | CMSIS · DSP 中的 arm_rfft_fast_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_init_f16.c` | CMSIS · DSP 中的 arm_rfft_fast_init_f16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_init_f32.c` | CMSIS · DSP 中的 arm_rfft_fast_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_init_f64.c` | CMSIS · DSP 中的 arm_rfft_fast_init_f64 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_init_f32.c` | CMSIS · DSP 中的 arm_rfft_init_f32 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_init_q15.c` | CMSIS · DSP 中的 arm_rfft_init_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_init_q31.c` | CMSIS · DSP 中的 arm_rfft_init_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_q15.c` | CMSIS · DSP 中的 arm_rfft_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_q31.c` | CMSIS · DSP 中的 arm_rfft_q31 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/TransformFunctions.c` | CMSIS · DSP 中的 TransformFunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/DSP/Source/TransformFunctions/TransformFunctionsF16.c` | CMSIS · DSP 中的 TransformFunctionsF16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/cachel1_armv7.h` | CMSIS · Include 中的 cachel1_armv7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/cmsis_armcc.h` | CMSIS · Include 中的 cmsis_armcc 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/cmsis_armclang_ltm.h` | CMSIS · Include 中的 cmsis_armclang_ltm 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/cmsis_armclang.h` | CMSIS · Include 中的 cmsis_armclang 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/cmsis_compiler.h` | CMSIS · Include 中的 cmsis_compiler 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/cmsis_gcc.h` | CMSIS · Include 中的 cmsis_gcc 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/cmsis_iccarm.h` | CMSIS · Include 中的 cmsis_iccarm 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/cmsis_version.h` | CMSIS · Include 中的 cmsis_version 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_armv81mml.h` | CMSIS · Include 中的 core_armv81mml 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_armv8mbl.h` | CMSIS · Include 中的 core_armv8mbl 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_armv8mml.h` | CMSIS · Include 中的 core_armv8mml 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_cm0.h` | CMSIS · Include 中的 core_cm0 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_cm0plus.h` | CMSIS · Include 中的 core_cm0plus 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_cm1.h` | CMSIS · Include 中的 core_cm1 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_cm23.h` | CMSIS · Include 中的 core_cm23 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_cm3.h` | CMSIS · Include 中的 core_cm3 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_cm33.h` | CMSIS · Include 中的 core_cm33 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_cm35p.h` | CMSIS · Include 中的 core_cm35p 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_cm4.h` | CMSIS · Include 中的 core_cm4 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_cm55.h` | CMSIS · Include 中的 core_cm55 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_cm7.h` | CMSIS · Include 中的 core_cm7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_cm85.h` | CMSIS · Include 中的 core_cm85 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_sc000.h` | CMSIS · Include 中的 core_sc000 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_sc300.h` | CMSIS · Include 中的 core_sc300 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/core_starmc1.h` | CMSIS · Include 中的 core_starmc1 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/mpu_armv7.h` | CMSIS · Include 中的 mpu_armv7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/mpu_armv8.h` | CMSIS · Include 中的 mpu_armv8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/pac_armv81.h` | CMSIS · Include 中的 pac_armv81 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/pmu_armv8.h` | CMSIS · Include 中的 pmu_armv8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/Include/tz_context.h` | CMSIS · Include 中的 tz_context 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Include/arm_nn_math_types.h` | CMSIS · NN 中的 arm_nn_math_types 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Include/arm_nn_tables.h` | CMSIS · NN 中的 arm_nn_tables 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Include/arm_nn_types.h` | CMSIS · NN 中的 arm_nn_types 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Include/arm_nnfunctions.h` | CMSIS · NN 中的 arm_nnfunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Include/arm_nnsupportfunctions.h` | CMSIS · NN 中的 arm_nnsupportfunctions 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.c` | CMSIS · NN 中的 arm_nn_activations_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.c` | CMSIS · NN 中的 arm_nn_activations_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.c` | CMSIS · NN 中的 arm_relu_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.c` | CMSIS · NN 中的 arm_relu_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ActivationFunctions/arm_relu6_s8.c` | CMSIS · NN 中的 arm_relu6_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/BasicMathFunctions/arm_elementwise_add_s16.c` | CMSIS · NN 中的 arm_elementwise_add_s16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/BasicMathFunctions/arm_elementwise_add_s8.c` | CMSIS · NN 中的 arm_elementwise_add_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/BasicMathFunctions/arm_elementwise_mul_s16.c` | CMSIS · NN 中的 arm_elementwise_mul_s16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/BasicMathFunctions/arm_elementwise_mul_s8.c` | CMSIS · NN 中的 arm_elementwise_mul_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_w.c` | CMSIS · NN 中的 arm_concatenation_s8_w 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_x.c` | CMSIS · NN 中的 arm_concatenation_s8_x 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_y.c` | CMSIS · NN 中的 arm_concatenation_s8_y 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_z.c` | CMSIS · NN 中的 arm_concatenation_s8_z 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_1_x_n_s8.c` | CMSIS · NN 中的 arm_convolve_1_x_n_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_1x1_HWC_q7_fast_nonsquare.c` | CMSIS · NN 中的 arm_convolve_1x1_HWC_q7_fast_nonsquare 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_1x1_s8_fast.c` | CMSIS · NN 中的 arm_convolve_1x1_s8_fast 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_fast_s16.c` | CMSIS · NN 中的 arm_convolve_fast_s16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q15_basic.c` | CMSIS · NN 中的 arm_convolve_HWC_q15_basic 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q15_fast_nonsquare.c` | CMSIS · NN 中的 arm_convolve_HWC_q15_fast_nonsquare 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q15_fast.c` | CMSIS · NN 中的 arm_convolve_HWC_q15_fast 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_basic_nonsquare.c` | CMSIS · NN 中的 arm_convolve_HWC_q7_basic_nonsquare 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_basic.c` | CMSIS · NN 中的 arm_convolve_HWC_q7_basic 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_fast_nonsquare.c` | CMSIS · NN 中的 arm_convolve_HWC_q7_fast_nonsquare 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_fast.c` | CMSIS · NN 中的 arm_convolve_HWC_q7_fast 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_RGB.c` | CMSIS · NN 中的 arm_convolve_HWC_q7_RGB 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_s16.c` | CMSIS · NN 中的 arm_convolve_s16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_s8.c` | CMSIS · NN 中的 arm_convolve_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_wrapper_s16.c` | CMSIS · NN 中的 arm_convolve_wrapper_s16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_wrapper_s8.c` | CMSIS · NN 中的 arm_convolve_wrapper_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_3x3_s8.c` | CMSIS · NN 中的 arm_depthwise_conv_3x3_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_s16.c` | CMSIS · NN 中的 arm_depthwise_conv_s16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_s8_opt.c` | CMSIS · NN 中的 arm_depthwise_conv_s8_opt 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_s8.c` | CMSIS · NN 中的 arm_depthwise_conv_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_u8_basic_ver1.c` | CMSIS · NN 中的 arm_depthwise_conv_u8_basic_ver1 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_wrapper_s8.c` | CMSIS · NN 中的 arm_depthwise_conv_wrapper_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_separable_conv_HWC_q7_nonsquare.c` | CMSIS · NN 中的 arm_depthwise_separable_conv_HWC_q7_nonsquare 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_separable_conv_HWC_q7.c` | CMSIS · NN 中的 arm_depthwise_separable_conv_HWC_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_depthwise_conv_s8_core.c` | CMSIS · NN 中的 arm_nn_depthwise_conv_s8_core 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_q7_q15_reordered.c` | CMSIS · NN 中的 arm_nn_mat_mult_kernel_q7_q15_reordered 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_q7_q15.c` | CMSIS · NN 中的 arm_nn_mat_mult_kernel_q7_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_s8_s16_reordered.c` | CMSIS · NN 中的 arm_nn_mat_mult_kernel_s8_s16_reordered 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_s8_s16.c` | CMSIS · NN 中的 arm_nn_mat_mult_kernel_s8_s16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_s8.c` | CMSIS · NN 中的 arm_nn_mat_mult_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_mat_q7_vec_q15_opt.c` | CMSIS · NN 中的 arm_fully_connected_mat_q7_vec_q15_opt 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_mat_q7_vec_q15.c` | CMSIS · NN 中的 arm_fully_connected_mat_q7_vec_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q15_opt.c` | CMSIS · NN 中的 arm_fully_connected_q15_opt 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q15.c` | CMSIS · NN 中的 arm_fully_connected_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q7_opt.c` | CMSIS · NN 中的 arm_fully_connected_q7_opt 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q7.c` | CMSIS · NN 中的 arm_fully_connected_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_s16.c` | CMSIS · NN 中的 arm_fully_connected_s16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_s8.c` | CMSIS · NN 中的 arm_fully_connected_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_nn_accumulate_q7_to_q15.c` | CMSIS · NN 中的 arm_nn_accumulate_q7_to_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_nn_add_q7.c` | CMSIS · NN 中的 arm_nn_add_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_nn_depthwise_conv_nt_t_padded_s8.c` | CMSIS · NN 中的 arm_nn_depthwise_conv_nt_t_padded_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_nn_depthwise_conv_nt_t_s8.c` | CMSIS · NN 中的 arm_nn_depthwise_conv_nt_t_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mat_mul_core_1x_s8.c` | CMSIS · NN 中的 arm_nn_mat_mul_core_1x_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mat_mul_core_4x_s8.c` | CMSIS · NN 中的 arm_nn_mat_mul_core_4x_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mat_mul_kernel_s16.c` | CMSIS · NN 中的 arm_nn_mat_mul_kernel_s16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mat_mult_nt_t_s8.c` | CMSIS · NN 中的 arm_nn_mat_mult_nt_t_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mult_q15.c` | CMSIS · NN 中的 arm_nn_mult_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mult_q7.c` | CMSIS · NN 中的 arm_nn_mult_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_nn_vec_mat_mult_t_s16.c` | CMSIS · NN 中的 arm_nn_vec_mat_mult_t_s16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_nn_vec_mat_mult_t_s8.c` | CMSIS · NN 中的 arm_nn_vec_mat_mult_t_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_nn_vec_mat_mult_t_svdf_s8.c` | CMSIS · NN 中的 arm_nn_vec_mat_mult_t_svdf_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_nntables.c` | CMSIS · NN 中的 arm_nntables 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_no_shift.c` | CMSIS · NN 中的 arm_q7_to_q15_no_shift 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_reordered_no_shift.c` | CMSIS · NN 中的 arm_q7_to_q15_reordered_no_shift 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_reordered_with_offset.c` | CMSIS · NN 中的 arm_q7_to_q15_reordered_with_offset 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_with_offset.c` | CMSIS · NN 中的 arm_q7_to_q15_with_offset 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/PoolingFunctions/arm_avgpool_s16.c` | CMSIS · NN 中的 arm_avgpool_s16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/PoolingFunctions/arm_avgpool_s8.c` | CMSIS · NN 中的 arm_avgpool_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/PoolingFunctions/arm_max_pool_s16.c` | CMSIS · NN 中的 arm_max_pool_s16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/PoolingFunctions/arm_max_pool_s8.c` | CMSIS · NN 中的 arm_max_pool_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/PoolingFunctions/arm_pool_q7_HWC.c` | CMSIS · NN 中的 arm_pool_q7_HWC 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/ReshapeFunctions/arm_reshape_s8.c` | CMSIS · NN 中的 arm_reshape_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/SoftmaxFunctions/arm_nn_softmax_common_s8.c` | CMSIS · NN 中的 arm_nn_softmax_common_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_q15.c` | CMSIS · NN 中的 arm_softmax_q15 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_q7.c` | CMSIS · NN 中的 arm_softmax_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_s16.c` | CMSIS · NN 中的 arm_softmax_s16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_s8_s16.c` | CMSIS · NN 中的 arm_softmax_s8_s16 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_s8.c` | CMSIS · NN 中的 arm_softmax_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_u8.c` | CMSIS · NN 中的 arm_softmax_u8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_with_batch_q7.c` | CMSIS · NN 中的 arm_softmax_with_batch_q7 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/SVDFunctions/arm_svdf_s8.c` | CMSIS · NN 中的 arm_svdf_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/NN/Source/SVDFunctions/arm_svdf_state_s16_s8.c` | CMSIS · NN 中的 arm_svdf_state_s16_s8 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/RTOS/Template/cmsis_os.h` | CMSIS · RTOS 中的 cmsis_os 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/RTOS2/Include/cmsis_os2.h` | CMSIS · RTOS2 中的 cmsis_os2 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/RTOS2/Include/os_tick.h` | CMSIS · RTOS2 中的 os_tick 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/RTOS2/Source/os_systick.c` | CMSIS · RTOS2 中的 os_systick 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/RTOS2/Source/os_tick_gtim.c` | CMSIS · RTOS2 中的 os_tick_gtim 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/RTOS2/Source/os_tick_ptim.c` | CMSIS · RTOS2 中的 os_tick_ptim 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/RTOS2/Template/cmsis_os.h` | CMSIS · RTOS2 中的 cmsis_os 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/CMSIS/RTOS2/Template/cmsis_os1.c` | CMSIS · RTOS2 中的 cmsis_os1 接口、实现或示例 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h` | ST HAL · stm32_hal_legacy 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32f4xx_hal_can_legacy.h` | ST HAL · hal_can_legacy 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32f4xx_hal_eth_legacy.h` | ST HAL · hal_eth_legacy 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32_assert_template.h` | ST HAL · stm32_assert_template 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc_ex.h` | ST HAL · hal_adc_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h` | ST HAL · hal_adc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_can.h` | ST HAL · hal_can 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cec.h` | ST HAL · hal_cec 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_conf_template.h` | ST HAL · hal_conf_template 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h` | ST HAL · hal_cortex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_crc.h` | ST HAL · hal_crc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cryp_ex.h` | ST HAL · hal_cryp_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cryp.h` | ST HAL · hal_cryp 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac_ex.h` | ST HAL · hal_dac_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac.h` | ST HAL · hal_dac 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dcmi_ex.h` | ST HAL · hal_dcmi_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dcmi.h` | ST HAL · hal_dcmi 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h` | ST HAL · hal_def 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dfsdm.h` | ST HAL · hal_dfsdm 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma_ex.h` | ST HAL · hal_dma_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h` | ST HAL · hal_dma 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma2d.h` | ST HAL · hal_dma2d 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dsi.h` | ST HAL · hal_dsi 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_eth.h` | ST HAL · hal_eth 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_exti.h` | ST HAL · hal_exti 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h` | ST HAL · hal_flash_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ramfunc.h` | ST HAL · hal_flash_ramfunc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h` | ST HAL · hal_flash 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_fmpi2c_ex.h` | ST HAL · hal_fmpi2c_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_fmpi2c.h` | ST HAL · hal_fmpi2c 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_fmpsmbus_ex.h` | ST HAL · hal_fmpsmbus_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_fmpsmbus.h` | ST HAL · hal_fmpsmbus 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h` | ST HAL · hal_gpio_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h` | ST HAL · hal_gpio 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_hash_ex.h` | ST HAL · hal_hash_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_hash.h` | ST HAL · hal_hash 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_hcd.h` | ST HAL · hal_hcd 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c_ex.h` | ST HAL · hal_i2c_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h` | ST HAL · hal_i2c 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2s_ex.h` | ST HAL · hal_i2s_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2s.h` | ST HAL · hal_i2s 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_irda.h` | ST HAL · hal_irda 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_iwdg.h` | ST HAL · hal_iwdg 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_lptim.h` | ST HAL · hal_lptim 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_ltdc_ex.h` | ST HAL · hal_ltdc_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_ltdc.h` | ST HAL · hal_ltdc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_mmc.h` | ST HAL · hal_mmc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_nand.h` | ST HAL · hal_nand 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_nor.h` | ST HAL · hal_nor 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pccard.h` | ST HAL · hal_pccard 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pcd_ex.h` | ST HAL · hal_pcd_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pcd.h` | ST HAL · hal_pcd 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h` | ST HAL · hal_pwr_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h` | ST HAL · hal_pwr 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_qspi.h` | ST HAL · hal_qspi 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h` | ST HAL · hal_rcc_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h` | ST HAL · hal_rcc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rng.h` | ST HAL · hal_rng 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc_ex.h` | ST HAL · hal_rtc_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h` | ST HAL · hal_rtc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_sai_ex.h` | ST HAL · hal_sai_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_sai.h` | ST HAL · hal_sai 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_sd.h` | ST HAL · hal_sd 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_sdram.h` | ST HAL · hal_sdram 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_smartcard.h` | ST HAL · hal_smartcard 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_smbus.h` | ST HAL · hal_smbus 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_spdifrx.h` | ST HAL · hal_spdifrx 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_spi.h` | ST HAL · hal_spi 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_sram.h` | ST HAL · hal_sram 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim_ex.h` | ST HAL · hal_tim_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h` | ST HAL · hal_tim 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h` | ST HAL · hal_uart 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_usart.h` | ST HAL · hal_usart 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_wwdg.h` | ST HAL · hal_wwdg 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h` | ST HAL · hal 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_adc.h` | ST HAL · ll_adc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_bus.h` | ST HAL · ll_bus 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_cortex.h` | ST HAL · ll_cortex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_crc.h` | ST HAL · ll_crc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_dac.h` | ST HAL · ll_dac 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_dma.h` | ST HAL · ll_dma 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_dma2d.h` | ST HAL · ll_dma2d 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_exti.h` | ST HAL · ll_exti 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_fmc.h` | ST HAL · ll_fmc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_fmpi2c.h` | ST HAL · ll_fmpi2c 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_fsmc.h` | ST HAL · ll_fsmc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_gpio.h` | ST HAL · ll_gpio 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_i2c.h` | ST HAL · ll_i2c 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_iwdg.h` | ST HAL · ll_iwdg 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_lptim.h` | ST HAL · ll_lptim 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h` | ST HAL · ll_pwr 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h` | ST HAL · ll_rcc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rng.h` | ST HAL · ll_rng 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rtc.h` | ST HAL · ll_rtc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_sdmmc.h` | ST HAL · ll_sdmmc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_spi.h` | ST HAL · ll_spi 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h` | ST HAL · ll_system 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_tim.h` | ST HAL · ll_tim 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_usart.h` | ST HAL · ll_usart 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_usb.h` | ST HAL · ll_usb 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_utils.h` | ST HAL · ll_utils 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_wwdg.h` | ST HAL · ll_wwdg 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c` | ST HAL · hal_adc_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c` | ST HAL · hal_adc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c` | ST HAL · hal_can 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cec.c` | ST HAL · hal_cec 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c` | ST HAL · hal_cortex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c` | ST HAL · hal_crc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cryp_ex.c` | ST HAL · hal_cryp_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cryp.c` | ST HAL · hal_cryp 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac_ex.c` | ST HAL · hal_dac_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac.c` | ST HAL · hal_dac 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dcmi_ex.c` | ST HAL · hal_dcmi_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dcmi.c` | ST HAL · hal_dcmi 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dfsdm.c` | ST HAL · hal_dfsdm 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c` | ST HAL · hal_dma_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c` | ST HAL · hal_dma 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma2d.c` | ST HAL · hal_dma2d 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dsi.c` | ST HAL · hal_dsi 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_eth.c` | ST HAL · hal_eth 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c` | ST HAL · hal_exti 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c` | ST HAL · hal_flash_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c` | ST HAL · hal_flash_ramfunc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c` | ST HAL · hal_flash 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_fmpi2c_ex.c` | ST HAL · hal_fmpi2c_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_fmpi2c.c` | ST HAL · hal_fmpi2c 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_fmpsmbus_ex.c` | ST HAL · hal_fmpsmbus_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_fmpsmbus.c` | ST HAL · hal_fmpsmbus 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c` | ST HAL · hal_gpio 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hash_ex.c` | ST HAL · hal_hash_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hash.c` | ST HAL · hal_hash 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hcd.c` | ST HAL · hal_hcd 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c` | ST HAL · hal_i2c_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c` | ST HAL · hal_i2c 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s_ex.c` | ST HAL · hal_i2s_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s.c` | ST HAL · hal_i2s 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_irda.c` | ST HAL · hal_irda 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_iwdg.c` | ST HAL · hal_iwdg 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_lptim.c` | ST HAL · hal_lptim 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_ltdc_ex.c` | ST HAL · hal_ltdc_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_ltdc.c` | ST HAL · hal_ltdc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_mmc.c` | ST HAL · hal_mmc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_msp_template.c` | ST HAL · hal_msp_template 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_nand.c` | ST HAL · hal_nand 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_nor.c` | ST HAL · hal_nor 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pccard.c` | ST HAL · hal_pccard 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c` | ST HAL · hal_pcd_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c` | ST HAL · hal_pcd 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c` | ST HAL · hal_pwr_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c` | ST HAL · hal_pwr 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_qspi.c` | ST HAL · hal_qspi 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c` | ST HAL · hal_rcc_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c` | ST HAL · hal_rcc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rng.c` | ST HAL · hal_rng 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c` | ST HAL · hal_rtc_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c` | ST HAL · hal_rtc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sai_ex.c` | ST HAL · hal_sai_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sai.c` | ST HAL · hal_sai 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c` | ST HAL · hal_sd 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sdram.c` | ST HAL · hal_sdram 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_smartcard.c` | ST HAL · hal_smartcard 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_smbus.c` | ST HAL · hal_smbus 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spdifrx.c` | ST HAL · hal_spdifrx 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c` | ST HAL · hal_spi 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sram.c` | ST HAL · hal_sram 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c` | ST HAL · hal_tim_ex 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c` | ST HAL · hal_tim 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_timebase_rtc_alarm_template.c` | ST HAL · hal_timebase_rtc_alarm_template 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_timebase_rtc_wakeup_template.c` | ST HAL · hal_timebase_rtc_wakeup_template 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_timebase_tim_template.c` | ST HAL · hal_timebase_tim_template 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c` | ST HAL · hal_uart 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c` | ST HAL · hal_usart 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_wwdg.c` | ST HAL · hal_wwdg 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c` | ST HAL · hal 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c` | ST HAL · ll_adc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_crc.c` | ST HAL · ll_crc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dac.c` | ST HAL · ll_dac 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dma.c` | ST HAL · ll_dma 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dma2d.c` | ST HAL · ll_dma2d 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_exti.c` | ST HAL · ll_exti 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fmc.c` | ST HAL · ll_fmc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fmpi2c.c` | ST HAL · ll_fmpi2c 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fsmc.c` | ST HAL · ll_fsmc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_gpio.c` | ST HAL · ll_gpio 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_i2c.c` | ST HAL · ll_i2c 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_lptim.c` | ST HAL · ll_lptim 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_pwr.c` | ST HAL · ll_pwr 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_rcc.c` | ST HAL · ll_rcc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_rng.c` | ST HAL · ll_rng 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_rtc.c` | ST HAL · ll_rtc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c` | ST HAL · ll_sdmmc 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_spi.c` | ST HAL · ll_spi 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_tim.c` | ST HAL · ll_tim 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usart.c` | ST HAL · ll_usart 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c` | ST HAL · ll_usb 外设接口或实现 | 教师锁死合同/骨架 |
| `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_utils.c` | ST HAL · ll_utils 外设接口或实现 | 教师锁死合同/骨架 |

</details>
