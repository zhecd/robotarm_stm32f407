# 数据流与中断边界

## 正常运动命令

```text
上位机 CRLF 文本
  → BSP UART1 DMA / 行缓冲
  → G-code 解析器：GCodeFrame_t
  → G-code 执行器：命令语义
  → 运动学：JointTarget_t
  → 规划器：MotionFrame_t 序列
  → 运动引擎队列
  → TIM6 50 kHz 中断
  → 关节设备
  → STEP/DIR 驱动
  → TMC2209 与步进电机
```

`ok` 表示 G0/G1 已被接受并进入运动队列。`M400` 返回 `ok` 才表示当前队列已完成。详细协议以 `serial-protocol.md` 为准。

## 测量与安全反馈

```text
AS5600
  → AS5600 Driver（I2C 原始角度）
  → Joint Device（连续电机角）
  → Closed-loop / StateService
  → 关节角和编码器有效性快照
  → SafetyService
  → 允许运动，或锁存故障并停止 MotionService
```

限位开关的 GPIO 边沿只表示硬件事实。App ISR 将引脚号转换为事件，并转交给安全相关服务处理。ISR 中不得执行串口打印、回零、轨迹计算或阻塞等待。

## 状态快照

后续服务化后的状态读取采用“完整快照 + generation”方式。写入端在短临界区内替换完整状态并递增 generation；读取端复制完整快照。临界区内禁止 I2C、UART、`printf`、轨迹计算和等待队列空间。
