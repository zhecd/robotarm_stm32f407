# 分层依赖规则

## 主调用链

```text
App → Service → Device → Driver → BSP/HAL → Hardware
```

`Domain` 和 `Platform` 是侧向能力：Domain 提供无硬件依赖的机器人算法，Platform 提供时间、临界区和调度基础能力。两者不属于主调用链中的额外业务层。

## 禁止依赖

| 模块 | 禁止直接依赖 |
|---|---|
| Domain | `stm32f4xx`、`HAL_`、CMSIS、FreeRTOS、Platform、BSP、Device、Service、App |
| Service | `HAL_`、GPIO/TIM/UART 句柄、GPIO 引脚宏、`__disable_irq()` |
| Device | G-code、XYZ 轨迹、回零策略、故障锁存和串口打印 |
| Driver | 关节名称、Home Pose、XYZ、G-code 和安全故障 |
| BSP | 机械臂关节、末端坐标和业务策略 |

## 编译期边界

本工程使用按目标划分的 CMake 包含目录，而不是把全部目录添加到每一个目标。所有手写代码位于 `RobotArm/`。`arm_app` 只能看到 App、Service、Domain 和 Platform 的公开接口；硬件相关头文件仅提供给 `arm_app_adapters`。`arm_service` 可以使用 Device、Domain 和 Platform 的公开接口，但运动控制内部头文件仅在 `RobotArm/Service/motion/internal/` 中可见。

`scripts/check_architecture_dependencies.py` 会额外检查 `RobotArm/Domain`、`RobotArm/Service` 与普通 App 源文件。`RobotArm/App/adapters` 是唯一允许包含 HAL、BSP 或 Device 头文件的 App 子目录。

因此，新增模块时应先判断其对外接口属于哪一层，再将实现文件加入 `RobotArm/` 下的对应目录与 CMake 目标。若某文件必须访问 STM32 句柄、GPIO 引脚或 HAL 回调，应放在 BSP、Driver、Device 或 `RobotArm/App/adapters/`，而不应放入普通 App、Service 或 Domain 源文件。

## 调用规则

1. App 负责初始化、协作式任务调度、ISR 分发和输入/输出适配，不保存业务状态。
2. Service 负责业务状态与业务决策，并通过 Device 操作机器人部件。
3. Device 只发布硬件事实和执行硬件动作，不决定运动许可或故障处置。
4. Driver 只封装芯片协议和时序；BSP 是 STM32 外设句柄、GPIO 映射和 DMA 资源的唯一归属。
5. 普通 STEP/DIR 输出最终只能从 MotionService 发出。故障锁存和故障解除最终只能由 SafetyService 完成。
