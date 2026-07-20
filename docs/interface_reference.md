# 服务接口参考

本文档说明用于模块间调用的主要公开接口。`internal/` 目录中的接口不属于公开调用面。

## PlanningService

| 项目 | 说明 |
|---|---|
| 职责 | 接收笛卡尔路径规划请求，并驱动内部规划器完成路径校验和运动帧生成。 |
| 公开头文件 | `RobotArm/Service/include/planning_service.h` |
| 调用者 | CommandService |
| 向下依赖 | `Ctrl_Planner`、MotionService 公开接口 |
| 上下文 | 在 App 主循环中调用，不在中断中调用。 |

主要接口：

```c
ErrorCode_t PlanningService_StartLine(float x, float y, float z,
                                      uint32_t duration_ms);
ErrorCode_t PlanningService_StartTeleopStep(float dx, float dy, float dz);
void PlanningService_Service(void);
bool PlanningService_TakeStartResult(ErrorCode_t *out_result);
```

非零长度直线运动先返回 `ERR_PENDING`。调用方应持续运行 `PlanningService_Service()`，再使用 `PlanningService_TakeStartResult()` 取得路径校验结果。

## MotionService

| 项目 | 说明 |
|---|---|
| 职责 | 提交运动帧、驱动 TIM6 STEP 输出、管理限位事件、理论步数和闭环补偿。 |
| 公开头文件 | `RobotArm/Service/include/motion_service.h` |
| 调用者 | PlanningService、AppRuntime、App ISR adapter |
| 向下依赖 | Motion internal、SafetyService、Platform、Device |
| 上下文 | `MotionService_OnStepTickFromISR()` 只能由 TIM6 ISR adapter 调用；其他接口在主循环调用。 |

`MotionService_SubmitFrame()` 会再次检查 `SafetyService_IsMotionAllowed()`。因此即使规划已经开始，故障锁存后的新运动帧也不会进入执行队列。

```c
bool MotionService_SubmitFrame(const MotionFrame_t *frame);
bool MotionService_TakeFaultEvent(MotionFaultReason_t *out_reason);
void MotionService_AbortForFault(MotionFaultReason_t reason);
```

`MotionService_AbortForFault()` 用于 Motion 域检测到无法继续安全执行的内部错误。它立即停止 STEP 输出，并生成一个待 AppRuntime 处理的故障事件。

## SafetyService

| 项目 | 说明 |
|---|---|
| 职责 | 锁存安全故障，保存回零状态和运动许可。 |
| 公开头文件 | `RobotArm/Service/include/safety_service.h` |
| 调用者 | AppRuntime、CommandService、MotionService |
| 向下依赖 | Platform 临界区接口 |
| 上下文 | 主循环调用；状态读取可在需要时获得一致快照。 |

`SafetyService_ReportLimitSwitch()`、`SafetyService_ReportEncoderFailure()`、`SafetyService_ReportSoftLimit()`、`SafetyService_ReportQueueTimeout()` 和 `SafetyService_ReportControlDivergence()` 均锁存故障并禁止后续运动。它们不再直接停止 MotionService。

## CommandService

| 项目 | 说明 |
|---|---|
| 职责 | 仲裁 G-code、PS2 和路径点 FIFO；维护规划位置和进给速度。 |
| 公开头文件 | `RobotArm/Service/include/command_service.h` |
| 调用者 | App G-code executor、App teleop |
| 向下依赖 | PlanningService、MotionService、SafetyService、GripperService |
| 上下文 | 主循环调用。 |

CommandService 只依赖 `PlanningService`，不访问 `Ctrl_Planner`。规划命令可能返回 `ERR_PENDING` 或 `ERR_QUEUED`；具体串口回复规则见 [serial-protocol.md](serial-protocol.md)。

## 板级共享接口

`BSP_LED_Init()` 只初始化 LED 的软件状态和输出电平。GPIO 模式、时钟和 EXTI 配置由 CubeMX 启动流程中的 `MX_GPIO_Init()` 统一完成，BSP 不重复初始化整块 GPIO。

三轴步进驱动器共用一个有效低电平的 `EN` 引脚，因此驱动接口为：

```c
bool Drv_Stepper_EnableAll(bool enable);
```

`Dev_Joint_EnableAll()` 是设备层的对应语义接口。项目不存在单轴使能的硬件能力，不应以单轴指针参数伪装为单轴使能。

夹爪配置使用 `GRIPPER_OPEN_ANGLE_DEG` 与 `GRIPPER_CLOSE_ANGLE_DEG`。当前数值分别保留原先 M3 打开和 M5 关闭时的实际舵机角度，因此本次命名调整不改变夹爪的机械动作。
