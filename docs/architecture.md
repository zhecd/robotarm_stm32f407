# 软件架构

## 架构目标

本项目采用“主调用链 + 侧向能力”的分层方式。主调用链描述一次机器人命令如何抵达硬件；Domain 和 Platform 提供被多个层复用的算法与运行时能力，但不承担额外的业务层级。

```text
Core / CubeMX → App → Service → Device → Driver → BSP / HAL → Hardware
                         ↑
                       Domain

                 Platform（时间、临界区、调度）
```

主调用方向为 `App → Service → Device → Driver → BSP/HAL`。反向信息通过只读状态快照、事件或服务查询接口返回，而不是由下层直接调用上层。

## 各层职责

| 层 | 负责内容 | 不负责内容 |
|---|---|---|
| Core / CubeMX | 时钟、外设初始化、启动文件和 HAL 回调入口 | 机器人业务流程 |
| App | 初始化顺序、协作式任务调度、ISR 分发、G-code/PS2 适配和状态上报 | 运行时业务状态、运动学、故障决策、硬件寄存器 |
| Service | 命令仲裁、运动、状态、安全、回零、夹爪和参数等业务能力 | HAL 句柄、GPIO 引脚和芯片寄存器 |
| Domain | 运动学、轨迹、插补、PID、数学限位和 G-code 语法 | 硬件、HAL、RTOS、当前模式和队列状态 |
| Device | 关节、夹爪、限位、操作输入、主机链路和指示灯等机器人部件 | G-code、XYZ 轨迹、故障策略 |
| Driver | AS5600、TMC2209、STEP/DIR、舵机 PWM、PS2 等芯片或协议 | 关节语义、Home Pose、G-code |
| BSP / HAL | STM32 外设句柄、引脚映射、DMA 和板级资源 | 关节、笛卡尔坐标和业务策略 |
| Platform | 时间、延时、临界区、事件和协作式调度接口 | 机器人业务与硬件引脚 |

## 当前实现

`main.c` 仅保留 CubeMX 初始化、`App_Init()`、`App_RunOnce()` 与 HAL 回调入口。App 层负责调度与协议适配，不直接调用 `Ctrl_MotionEngine` 或 `Ctrl_Planner`。

- `MotionService` 封装运动引擎初始化、STEP 定时器入口、限位开关事件、限位监控和故障原因查询。
- `CommandService` 负责 G-code 运动命令、规划器初始化、路径校验、轨迹帧生成及规划状态提交。
- `SafetyService` 锁存安全故障，并在成功回零与编码器校准后解除故障。
- `StateService` 保存编码器采样和关节角度快照，供 `M114` 和状态上报读取。

笛卡尔轨迹采用协作式处理：规划器先分批校验全部插补点，再分批向运动队列生成轨迹帧。这样既不会在发现中间点不可达后执行部分轨迹，也不会因长距离运动而长时间占用主循环。上电回零和 `M999` 恢复也使用同一非阻塞状态机；回零完成后才启动规划器、闭环和运动定时器。

## 源码目录

```text
App/                 应用初始化、任务调度、G-code 解析与 PS2 适配
App/include/         App 层公开接口
BSP/                 LED、UART 等板级资源及其公开接口
Device/              关节、夹爪、限位与输入设备抽象
Driver/              AS5600、TMC2209、STEP/DIR、舵机与 PS2 驱动
Domain/              运动学、Home Pose 与数学算法
Service/             命令、运动、安全、状态、回零、夹爪与控制服务
Platform/            时间、延时、临界区与兼容接口
Core/                仅保留 STM32CubeMX 生成的启动、外设与 HAL 配置
```

各层的公开头文件位于本层 `include/` 目录，源文件与其业务子目录位于同一层中。`Core/Inc` 不再承载用户维护的机器人模块。`Service/control` 通过 Platform 访问时间和临界区，不直接调用 HAL 或 CMSIS 临界区原语。夹爪 PWM 的定时器绑定属于 Device 初始化；夹爪服务只保留打开、关闭与空闲停止等设备语义。

## 关键规则

1. 只有 MotionService 可以进行普通 STEP/DIR 输出。
2. 只有 SafetyService 可以锁存或解除运动安全故障。
3. 只有 StateService 可以写入实测关节状态。
4. 只有 CommandService 可以仲裁 G-code、PS2 和内部命令。
5. Domain 必须能够脱离 STM32 HAL 在主机端独立编译和测试。
6. ISR 只转换硬件事件或执行短时硬实时步骤；ISR 中不得打印、回零、规划轨迹或等待队列。

状态所有权、数据流、依赖限制和命名规则分别见 `service_ownership.md`、`data_flow.md`、`dependency_rules.md` 与 `naming_conventions.md`。
