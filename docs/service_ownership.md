# 服务层状态与决策所有权

## 目的

本文件定义 `robotarm_stm32f407` 重构后的运行时状态和决策边界。每一项可变状态只能由一个服务写入，其他模块通过只读快照或服务接口读取。这一约束避免 G-code、规划器、闭环控制和中断路径同时维护同一份位置或故障状态。

## 所有权表

| 服务 | 唯一可写状态 | 唯一决策权 | 当前迁移状态 |
|---|---|---|---|
| `CommandService` | 活动命令、输入源、模式和命令 generation | G-code、PS2 和内部命令的仲裁 | 后续阶段 |
| `MotionService` | 已接受的规划位置、理论步数、运动队列和执行状态 | 正常 STEP/DIR 输出 | 后续阶段；当前由 `ctrl_motion_engine` 与 `ctrl_planner` 承担 |
| `StateService` | 编码器测量、连续角度、测量时间戳和有效性 | 发布测量快照 | 后续阶段；当前由 `ctrl_closed_loop` 承担 |
| `SafetyService` | 故障锁存、回零状态和运动许可 | 接受或拒绝运动、锁存或清除故障 | 后续阶段；当前由 `ctrl_motion_engine` 承担 |
| `HomingService` | 回零状态机上下文 | 回零过程推进 | 后续阶段；当前为同步 `Svc_Homing_Execute()` |
| `GripperService` | 夹爪服务状态 | 打开、关闭和空闲停止策略 | 已有 `svc_gripper` |

## 当前第一轮的约束

第一轮重构不改变运动参数、插补算法、回零流程或串口协议。其目标是将入口、任务调度和 HAL 回调移出 `main.c`，并建立后续服务化所需的目录、构建目标和依赖规则。

在这一阶段，`ctrl_motion_engine` 仍暂时保存队列与故障，`ctrl_closed_loop` 仍暂时保存编码器相关状态。这些兼容职责将在 `StateService`、`SafetyService` 和 `MotionService` 建立后迁移；在迁移完成前，不新增第二个写入者。
