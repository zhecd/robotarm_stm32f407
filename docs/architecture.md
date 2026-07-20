# 软件架构

本项目将 STM32CubeMX 生成代码与手写机械臂固件分离。`Core/` 负责时钟、外设初始化和 HAL 回调入口；`RobotArm/` 负责机械臂的运行逻辑。软件分层的目的，是让运动规划、执行、设备控制和安全状态各自具有明确的职责边界。

## 主调用链

```text
Core/main
  -> AppRuntime
      -> CommandService
          -> PlanningService
              -> Ctrl_Planner
                  -> MotionService
                      -> Ctrl_MotionEngine
                          -> Dev_Joint
                              -> BSP / Driver / HAL
```

`CommandService` 解释已经解析完成的 G-code 或 PS2 增量命令。`PlanningService` 是笛卡尔路径规划的公开入口，负责将请求转交给内部规划器。`MotionService` 负责运动帧提交、STEP 定时器入口、限位事件和闭环补偿；它是普通运动帧进入执行器的唯一服务入口。

## 故障链路

```text
限位开关 / 编码器故障 / 软限位 / 闭环发散
  -> Motion internal 立即停止 STEP 输出并清空队列
  -> MotionService 保存故障事件
  -> AppRuntime 取出事件并映射
  -> SafetyService 锁存故障、禁止新的运动命令
  -> M999 回零成功后，由 AppRuntime 清除 Motion 故障并恢复安全许可
```

立即停止由 Motion 域完成，不依赖 App 主循环的下一次调度。`SafetyService` 只拥有安全状态，不直接调用 `MotionService`，因此二者不存在反向服务依赖。

## 目录结构

```text
RobotArm/
├─ App/                         启动顺序、协作式调度、G-code/PS2 适配
│  └─ adapters/                 CubeMX、HAL、BSP、Device 的适配边界
├─ Algorithm/                   与 STM32 无关的运动学、Home Pose 和数学算法
├─ BSP/                         板级 LED、UART 与芯片驱动
│  └─ driver/                   AS5600、TMC2209、STEP/DIR、PWM 舵机、PS2
├─ Device/                      关节、夹爪、限位与输入设备语义
├─ Platform/                    时间、延时、临界区和兼容适配
└─ Service/
   ├─ command/                  G-code 和 PS2 命令仲裁、路径点 FIFO
   ├─ planning/                 规划服务
   │  └─ internal/              Ctrl_Planner 私有实现
   ├─ motion/                   运动服务
   │  └─ internal/              Ctrl_MotionEngine、Ctrl_ClosedLoop 私有实现
   ├─ safety/                   安全状态锁存
   ├─ state/                    编码器与关节状态快照
   ├─ homing/                   回零状态机
   └─ gripper/                  夹爪服务
```

公开头文件集中在 `RobotArm/*/include/`。`internal/` 中的头文件只供同一服务域的实现文件使用，不能被 App 或 CommandService 直接包含。

## 层职责

| 层 | 负责内容 | 不负责内容 |
|---|---|---|
| App | 初始化顺序、任务调度、协议适配、故障事件映射 | GPIO 寄存器、轨迹插补、故障状态存储 |
| Service | 命令、规划、运动、安全、状态、回零和夹爪业务 | HAL 句柄、GPIO 引脚和芯片寄存器 |
| Algorithm | 运动学、数学换算、Home Pose | HAL、RTOS、设备和业务服务 |
| Device | 关节、夹爪、限位、输入设备动作 | G-code、笛卡尔轨迹和安全策略 |
| BSP / Driver | 外设、引脚、DMA 和芯片通信时序 | 关节语义、运动许可和任务流程 |
| Platform | 时间、临界区和基础运行时能力 | 机械臂业务和硬件引脚 |

## 编译边界

CMake 为 `arm_algorithm`、`arm_platform`、`arm_bsp`、`arm_device`、`arm_service`、`arm_app` 和 `arm_app_adapters` 分别声明包含目录与链接依赖。这样可以在编译阶段发现跨层包含。

架构检查脚本还会验证以下约束：

- `CommandService` 不得包含 `ctrl_planner.h`。
- `Ctrl_Planner` 不得包含 `ctrl_motion_engine.h`。
- `SafetyService` 不得包含 `motion_service.h`。
- 普通 App 源文件不得包含内部实现、HAL、BSP 或 Device 头文件。
- `Algorithm` 不得依赖 HAL、BSP、Device、Service 或 App。

详细公开接口见 [interface_reference.md](interface_reference.md)，依赖规则见 [dependency_rules.md](dependency_rules.md)。
