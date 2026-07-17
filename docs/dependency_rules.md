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

## 调用规则

1. App 负责初始化、协作式任务调度、ISR 分发和输入/输出适配，不保存业务状态。
2. Service 负责业务状态与业务决策，并通过 Device 操作机器人部件。
3. Device 只发布硬件事实和执行硬件动作，不决定运动许可或故障处置。
4. Driver 只封装芯片协议和时序；BSP 是 STM32 外设句柄、GPIO 映射和 DMA 资源的唯一归属。
5. 普通 STEP/DIR 输出最终只能从 MotionService 发出。故障锁存和故障解除最终只能由 SafetyService 完成。
