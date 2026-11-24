# 公开接口与阅读入口

应用串口状态末尾新增 `APP_SERIAL_OUTPUT_LOCKED`，仅专用启动审计使用；既有状态编号不变。它代表驱动/反馈检查通过但执行器仍锁定，HOMED/ALLOWED均0；不参与READY周期调度。M112取消此阶段并转FAILED，M999显式重新检查后仍保留旧故障，直到将来真实回零/标定成功；标准构建默认 `ROBOTARM_STARTUP_AUDIT_MODE=0`，原正常启动功能不变。

本轮正常启动使用 `AppHardware_PrepareMotionDrivers`，经Device/BSP调用驱动Verified接口：EN始终禁用，主循环独占USART6，成功failed_node为0xFF，首个节点失败为0..2；IRQ锁存阻止准备时也为0xFF，应用优先报告过载。UART等待不进入临界区。

`Drv_TMC2209_ConfigNodeVerified`先读IFCNT、发送四项配置，再确认计数模256增加4及GCONF/CHOPCONF可写位读回；可选报告包含本次结果、读错误、失败寄存器和比较值，不用共享last-error。NODECONF/IHOLD_IRUN只写，通过IFCNT间接确认；不能据此推算实测安培。旧Checked接口仍仅确认发送。配置未确认时Safety追加DRIVER_CONFIG=10，保留旧编号0..9；M999显式重试后仍需反馈、回零/标定。

## 应用入口

- App_Init / App_RunOnce：正式启动及主循环；ROBOTARM_DIAGNOSTIC_MODE 选择独立诊断流程。
- AppSerial_Init / SetMode / Service：协议会话，调用不等待机械运动完成。
- AppSerial_TakeRecoveryRequest：把 M999 转交 AppRuntime 编排。
- AppSerial_TakeStopRequest：把本次M112事件交给AppRuntime，即使Safety首故障没有变化，仍取消检查/回零。
- AppHardware：可替换的硬件端口，公共类型不携带 HAL。
- AppHardware_StopActuators / MaskLimitInterrupt：可在ISR使用的停止和单轴屏蔽端口；早期EXTI不解引用未初始化的定时器。
- AppHardware_Init / PrepareMotionDrivers：初始化和配置时保持输出禁用；不隐含夹爪初始化或关节使能。AppHardware_InitGripper由正常入口在驱动确认、编码器就绪后调用一次，可能立即闭合夹爪。诊断/WAVE入口不调用它。

## 服务入口

| 接口 | 参数和结果 | 调用边界 |
|---|---|---|
| CommandService_RunGCode | GCodeFrame_t；返回成功、待校验、排队或错误 | 主循环；坐标 mm、F mm/min |
| CommandService_Service | 分次推进；结果由 TakeMoveResult/TakeQueuedMoveResult 取得 | 主循环；故障时撤销待处理动作 |
| CommandService_AbortPending | 清除路径点、规划工作和待读取结果 | 不能据此恢复位置参考 |
| PlanningService_StartLine | 目标 mm、总时长 ms | 异步校验；反复调用 Service 推进 |
| PlanningService_Cancel | 取消校验与帧生成 | 与 Motion 停止配合，恢复后须重新回零 |
| MotionService_SubmitFrame | MotionFrame_t：有符号电机微步、非零 tick | 无许可、故障、非法帧或队列满返回 false |
| MotionService_IsActionAllowed | bool；先确认到期限位，再合并本轮回零许可、Motion 故障和 IRQ 锁存 | 主循环；无 I2C、无补偿生成，不可在 ISR 调用 |
| CommandService_RunTeleopStep | dx/dy/dz：mm；受理后同步闭环理论目标 | 无许可或已有未完成短帧时 ERR_BUSY |
| Svc_Gripper_Open / Close | bool；true 仅表示设备调用被接受 | 主循环；检查完整动作许可，不表示机械到位 |
| MotionService_OnStepTickFromISR | 一个 STEP 时基节拍 | ISR；不打印、不 I2C、不等待 |
| MotionService_NotifyLimitSwitch | uint8_t 逻辑位图，bit0=M1、bit1=M2、bit2=M3 | ISR adapter 负责 GPIO 到位图转换 |
| MotionService_AbortForFault | 原因枚举 | 清除当前及未来 STEP，首个原因保持锁存 |
| MotionService_TakeFaultEvent | 一次事件 | 同一故障不重复发布，恢复后可发布新事件 |
| StateService_GetStatus | 完整状态副本 | 离开临界区后才做格式化与通信 |
| SafetyService_MarkHomed / ClearAfterSuccessfulHoming | 本轮回零/标定成功后建立许可 | 故障已使旧 homed 无效 |
| Svc_Homing_Start / Step / Cancel | 寻限位、退回、取消 | 主循环按时间推进，无阻塞回零循环 |

## 本次内部接口调整

TMC驱动新增`Drv_TMC2209_ConfigNodeVerifiedWithReference`，显式选择外部VREF或内部电流参考；所有兼容入口继续使用外部VREF。参考源选择不等于采样电阻选择，始终保留外部Rsense。确认报告增加本次原始GCONF/CHOPCONF读回；非法模式零总线访问，IFCNT不足或参考源位不符均不放行。驱动不控制EN/STEP，不以软件设定代替实际相电流测量。独立M1HOME3测试使用内部参考；正常G-code合同与115200串口不变。

LimitIrqGuard_OnEdgeFromISR / GetLatchMask：正常/WAVE4独立过载锁存；无在线清除入口。PlatformTime_OnTickFromISR由Core的SysTick调用，持续累计DWT周期差；NowUs返回32位微秒自然回绕值，时钟固定，SysTick不能停用超过一次CPU周期计数回绕。

`Dev_Joint_ConfigureDrivers()` 与 `Dev_Gripper_Init()` 不再接受 STM32 句柄；映射由 BSP 处理。Motion 的限位通知参数从 GPIO 编号改为逻辑轴位图。工程内全部调用同步更新；其他外部 C 调用方需按新头文件迁移。UART G-code 接口保留原命令并增加 M112/M115/M408。

电机轴 deg 与关节 deg 不可混用。StepsToDeg/DegToSteps 属于电机轴换算，UNITS_PER_DEGREE 已包含减速比。头文件中的中文合同优先于根据函数名推测语义。

本轮 Svc_Gripper_Open/Close 从 void 改为 bool，工程调用同步更新。MotionFaultReason_t 追加 INPUT_LOST=8，SafetyFault_t 追加 INPUT_LOST=9，原值不变；PS2 模式无效输入锁存故障，须重新回零，G-code 无手柄不触发此故障。首次启动夹爪初始化由 AppRuntime 单独编排，不属于普通夹爪命令。
