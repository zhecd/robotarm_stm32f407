# 模块职责与状态归属

| 模块 | 保存的状态 | 公开意图与边界 |
|---|---|---|
| AppSerial | 会话模式、待校验回复、M400 等待、恢复请求 | 解析和回复，不直接回零或访问 HAL |
| AppRuntime | 启动/回零/失败、恢复来源、模式切换请求 | 调度和故障映射，不拥有队列或 PID |
| AppTeleop | 手柄模式、采样周期、按键前值 | 提交遥操作增量与夹爪意图 |
| CommandService | 最后接受的参考位置、F、16 点 FIFO、异步结果 | 缺省参数继承、来源仲裁、撤销待执行动作 |
| PlanningService / Ctrl_Planner | 当前路径、校验索引、生成索引、队列等待时间 | 先完整校验，再流式生成帧；通过 Motion 公开接口提交 |
| MotionService / Ctrl_MotionEngine | 当前帧、63 帧可用队列、插补累计量、首个故障 | STEP 执行、限位暂停/消抖、一次性故障事件 |
| Ctrl_ClosedLoop | PID、滤波、误差、恢复轨迹、失败计数 | 保持原目标，补偿不改变规划理论位置 |
| SafetyService | homed、motion_allowed、首个故障 | 锁存与授权，不调用运动服务 |
| StateService | 单轴角度、有效性、时间戳、generation | 发布和复制反馈快照 |
| Svc_Homing | 寻限位/退回/完成/失败、逐轴进度 | 在专用启动/恢复阶段直接推进设备，每次调用工作量有界 |
| Device | 逻辑设备到具体器件的组合 | 公共接口不带 HAL 句柄和 GPIO 引脚 |
| BSP / driver | 外设绑定、传输缓冲和器件实例 | GPIO/I2C/UART/PWM/STEP 的具体操作 |

取消动作必须同时撤销 Command、Planner 和 Motion 的未来工作。普通清队列仅删除排队帧，保留当前帧；软件停止与故障使用 AbortForFault。安全状态清除必须发生在本轮回零和编码器标定成功之后。
