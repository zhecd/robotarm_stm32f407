# STM32 机械臂控制代码架构详解

## —— 四层解耦架构在 SCARA 教学机械臂中的实践

---

## 一、硬件概览

```
STM32F407VETx @ 168 MHz, bare metal (无 RTOS)
├── M1 (底座旋转):  TMC2209 步进驱动 + AS5600 编码器 (I2C1)
├── M2 (肩关节):    TMC2209 步进驱动 + AS5600 编码器 (I2C2)
├── M3 (肘关节):    TMC2209 步进驱动 + AS5600 编码器 (I2C3)
├── 夹爪:           PWM 舵机 (TIM2_CH2, PA1)
├── 通信:           PS2 无线手柄 (bit-bang SPI) + UART1 (G-code 上位机)
├── 回零:           3 路限位开关
└── 指示:           4 颗 LED (PC0-PC3)
```

---

## 二、四层架构总览

代码严格遵循 **APP → CONTROL → BSP → HAL** 单向依赖链。每一层只能向下调用, 绝不反向。

```
┌─────────────────────────────────────────────────────┐
│  APP (应用层)           │  业务编排                 │
│  app_teleop, app_gcode  │  手柄/G-code 指令分发      │
│  app_calibration        │  不知电机如何转动           │
├─────────────────────────┼───────────────────────────┤
│  CONTROL (控制层)       │  算法与决策                │
│  ctrl_planner           │  轨迹规划 (quintic smooth)  │
│  ctrl_kinematics        │  逆运动学求解               │
│  ctrl_motion_engine     │  Bresenham 脉冲引擎 (ISR)   │
│  ctrl_closed_loop       │  PID 闭环位置修正            │
│  ctrl_compensation      │  静态误差补偿                │
│  ctrl_ps2, ctrl_gripper │  设备抽象适配               │
├─────────────────────────┼───────────────────────────┤
│  BSP (板级支持层)       │  硬件驱动                   │
│  bsp_as5600, bsp_stepper│  编码器/步进/舵机/PWM       │
│  bsp_tmc2209, bsp_ps2   │  具体 GPIO/I2C/SPI/UART   │
│  bsp_homing, bsp_led    │  无业务逻辑                  │
├─────────────────────────┼───────────────────────────┤
│  HAL (硬件抽象层)        │  STM32CubeMX 自动生成       │
│  i2c, tim, usart, gpio  │  ST 官方驱动                │
└─────────────────────────────────────────────────────┘

          COMMON (公共层) — error_code, robot_config, common
          所有层都可以引用, 零业务依赖
```

### 核心规则

| 规则 | 说明 |
|------|------|
| **单向依赖** | APP → CONTROL → BSP → HAL, 不可反向 |
| **同级不可见** | APP 层 `app_teleop.c` 不知道 `app_gcode_exec.c` 的存在 |
| **头文件不出血** | CONTROL 的头文件不暴露 BSP 类型给上层 |
| **组装在 main.c** | 依赖注入 (DI) 集中在 main.c 完成, 模块内部不 `extern` 全局变量 |

---

## 三、逐层详解

### 3.1 COMMON 层 —— "零依赖"的基石

```
Core/Inc/common/
├── error_code.h      → ErrorCode_t 枚举 (ERR_OK, ERR_ENCODER_FAIL, ...)
├── robot_config.h    → 全部可调参数 (PID 增益、连杆长度、舵机角度...)
└── common.h          → 工具内联函数 (DegToSteps, AngleWrap180, CLAMP...)
```

**为什么这是最重要的层?** 它定义了整个项目的"共同语言":
- `robot_config.h` 是**唯一的调参入口**——修改任何参数只需改一个文件
- `AngleWrap180()` 将任意角度折叠到 [-180°, 180°], 是所有角度计算的基准
- `DegToSteps()` / `StepsToDeg()` 确保角度与步数转换的一致性

**设计原则**: COMMON 层不 include 任何 APP/CONTROL/BSP 头文件, 是纯正的 "基础数据类型 + 工具函数" 集合。

---

### 3.2 BSP 层 —— "硬件驱动, 无业务逻辑"

BSP 层每个模块的职责边界非常清晰:

```
模块              | 知道什么                | 不知道什么
──────────────────┼────────────────────────┼───────────────────
bsp_stepper       | Step/Dir/Enable 引脚     | 何时该走步、走多少步
bsp_as5600        | I2C 读取 + 多圈追踪       | PID、目标位置
bsp_gripper       | PWM 脉宽 → 舵机角度       | G-code M3/M5 指令
bsp_tmc2209       | UART 写 TMC2209 寄存器    | 细分、电流值含义
bsp_ps2           | bit-bang SPI 读手柄       | 摇杆 → 笛卡尔坐标映射
bsp_homing        | 限位开关触发 → 回退       | 回退位置对应哪个关节角度
bsp_led           | GPIO 读写                | LED 表示什么状态
bsp_uart1         | 环形缓冲 + 接收中断        | G-code 解析、协议
```

**关键**: 每个 BSP 模块都可以**独立测试**——给它 I2C 句柄就能读编码器, 给它 UART 句柄就能配置 TMC2209。

---

### 3.3 CONTROL 层 —— "算法核心, 不碰引脚"

这是整个项目的大脑, 分层最细:

#### 3.3.1 运动引擎 (`ctrl_motion_engine`) —— 50kHz 心跳

```c
// 锁定单 ring buffer (64 帧), 单生产者/单消费者
// TIM6 ISR @ 50kHz: Bresenham 三轴脉冲生成 + GPIO 直接输出
MotionFrame_t frame = { .delta_m1 = 100, .delta_m2 = -50, .delta_m3 = 0,
                        .total_ticks = 3200 };
Ctrl_MotionEngine_PushFrame(&frame);
// ISR 会自动在 3200 个 tick 内将步数均匀分布到各轴
```

**为什么需要运动引擎层？** 因为步进电机需要**精确的脉冲时序**。不能在上层用 `HAL_Delay` 凑——50kHz ISR 保证微秒级精度。上层只投递"相对运动帧", 不关心脉冲如何生成。

#### 3.3.2 运动学 (`ctrl_kinematics`) —— "知道手臂几何"

```c
typedef struct {
    float rot;   // 底座旋转角度 (°)
    float low;   // 肩关节角度 (°)
    float high;  // 肘关节角度 (°)
} RobotAngles_t;

// 笛卡尔坐标 → 关节角度
Ctrl_Kinematics_Solve(x, y, z, &angles);
// 关节角度 → 电机步数单位
Ctrl_Kinematics_ToMotorUnits(&angles, &units);
```

这层**只做数学**:
- 连杆长度、工具偏移全从 `robot_config.h` 读取
- 不知道电机型号、编码器、I2C
- 可以脱离硬件, 在 PC 上纯数学验证

#### 3.3.3 规划器 (`ctrl_planner`) —— "把目标变成帧"

```c
// G-code 运动: quintic smoothstep 插值
Ctrl_Planner_MoveLine(x, y, z, duration_ms);

// PS2 遥控: 单帧增量
Ctrl_Planner_TeleopStep(dx, dy, dz);
```

规划器的工作是:
1. 调用运动学求解目标关节角度
2. 计算当前 → 目标的步数差
3. 分割成多个小帧推入运动引擎

**不知道**是什么触发了运动 (G-code? PS2? 补偿?)

#### 3.3.4 PID 闭环 (`ctrl_closed_loop`) —— "守门员"

```c
// 50Hz 运行: 读编码器 → 算误差 → PID → 注入修正帧
void Ctrl_ClosedLoop_Update(void);

// 对外暴露编码器查询 (不暴露 AS5600_Dev_t*)
Ctrl_ClosedLoop_GetAxisAngle(axis, &deg);
Ctrl_ClosedLoop_SetAxisZero(axis);
```

**关键设计**:
- EMA 低通滤波消除编码器噪声
- 积分分离 (小误差积分, 大误差清零) 防积分饱和
- 自适应增益 (大误差强修正, 小误差保守)
- 冷却计时器防止修正帧过于密集
- **头文件不暴露 `AS5600_Dev_t*`** —— 上层不需要知道编码器是什么型号

#### 3.3.5 补偿模块 (`ctrl_compensation`) —— "最后一道关"

G-code 运动完成后, 迭代地将编码器驱动到精确目标:
1. 等待运动队列清空 (期间持续读编码器, 防止多圈丢失)
2. 快照理论步数作为绝对目标
3. 循环: 读编码器 → 算误差 → 推修正帧 → 等待执行
4. 死区检查 + 发散检测 (看门狗) + 编码器卡死检测

**重要**: 补偿模块的编码器访问全部通过 `Ctrl_ClosedLoop_GetAxisAngle()`, 不直接调 BSP。

#### 3.3.6 设备适配层 (`ctrl_ps2`, `ctrl_gripper`)

这两个是**薄包装** (thin wrapper / delegate pattern):

```c
// ctrl_gripper.c — 依赖注入示例
void Ctrl_Gripper_Init(TIM_HandleTypeDef *htim) {  // Timer 句柄由 main.c 注入
    BSP_Gripper_Init(BSP_Gripper_GetHandle(), htim, TIM_CHANNEL_2);
}
void Ctrl_Gripper_Open(void)  { BSP_Gripper_Open(BSP_Gripper_GetHandle()); }
void Ctrl_Gripper_Close(void) { BSP_Gripper_Close(BSP_Gripper_GetHandle()); }
```

**为什么需要薄包装？** 看起来只是转发, 但价值巨大:
1. APP 层不 include BSP 头文件 → 换硬件只改 CONTROL
2. 可以 mock 接口做单元测试
3. 函数签名是"语义化"的 (`Ctrl_Gripper_Open` vs `BSP_Gripper_Open(BSP_Gripper_GetHandle())`)

---

### 3.4 APP 层 —— "业务编排, 不碰硬件"

```
app_teleop.c       → PS2 手柄 → 摇杆→坐标映射 → Ctrl_Planner_TeleopStep
app_gcode_parser.c → 文本行解析 → GCodeFrame_t
app_gcode_exec.c   → G0/G1/M3/M5 指令分发
app_calibration.c  → 编码器归零 (Ctrl_ClosedLoop_SetAxisZero)
```

APP 层的特点:
- **零 BSP include** (经过 CONTROL 薄包装访问所有硬件)
- 只做**业务流程**, 不含算法
- 可以脱离硬件逻辑验证 (给 mock 数据)

---

### 3.5 main.c —— "组装根" (Composition Root)

```c
int main(void) {
    // 1. HAL 初始化 (CubeMX 生成)
    HAL_Init();
    SystemClock_Config();
    MX_GPIO/TIM/I2C/USART_Init();

    // 2. BSP 驱动初始化
    BSP_LED_Init();
    BSP_Stepper_Init();
    BSP_UART1_Init();

    // 3. CONTROL 薄包装初始化 (依赖注入)
    Ctrl_PS2_Init();
    Ctrl_Gripper_Init(&htim2);  // ← Timer 句柄在此注入
    BSP_AS5600_Init();

    // 4. 驱动参数配置
    BSP_Stepper_Enable(M1/M2/M3, true);
    BSP_TMC2209_ConfigNode(...);  // 细分、电流

    // 5. 回零 + 校准
    BSP_Homing_Execute();
    App_Calibration_Execute();

    // 6. 算法初始化
    Ctrl_MotionEngine_Init();
    Ctrl_Planner_Init(...);
    Ctrl_ClosedLoop_Init();

    // 7. 启动 ISR (50kHz 运动引擎)
    HAL_TIM_Base_Start_IT(&htim6);

    // 8. 主循环
    while (1) {
        if (mode == SYS_MODE_GCODE) {
            Task_EncoderRead();    // 50Hz 多圈跟踪
            Task_EncoderReport();  // 1Hz  状态报告
            Task_ClosedLoop();     // 50Hz PID 闭环
        }
        App_Teleop_Task();         // 100Hz PS2 遥控
        Ctrl_Gripper_IdleStop();   // 舵机空闲停机
        if (mode == SYS_MODE_GCODE) {
            Task_GCode();          // 串口指令处理
        }
    }
}
```

---

## 四、数据流全景 —— 一条 G-code 指令的旅程

以 `G1 X150 Y200 Z100 F300` 为例:

```
┌─ UART1 RX ───────────────────────────────────────────────────────┐
│ BSP_UART1_ReadLine()          [BSP]  环形缓冲读出一行文本          │
│        ↓                                                        │
│ App_GCodeParser_ParseLine()   [APP]  文本 → GCodeFrame_t 结构体   │
│        ↓                                                        │
│ App_GCodeExec_Run()           [APP]  switch(cmd) → RunLinearMove  │
│        ↓                                                        │
│ Ctrl_Planner_MoveLine()       [CTRL] 计算目标关节角, 分段帧        │
│        ↓                                                        │
│ Ctrl_Kinematics_Solve()       [CTRL] 笛卡尔 → 关节角度             │
│ Ctrl_Kinematics_ToMotorUnits() [CTRL] 关节角 → 步数单位            │
│        ↓                                                        │
│ Ctrl_MotionEngine_PushFrame() [CTRL] 帧入 ring buffer             │
│        ↓                                                        │
│ TIM6 ISR (50kHz)              [CTRL] Bresenham 算法 → 脉冲        │
│        ↓                                                        │
│ BSP_Stepper_Step()            [BSP]  GPIO 翻转 → 电机转动          │
│        ↓                                                        │
│ [运动完成, 队列清空]                                              │
│        ↓                                                        │
│ Ctrl_Compensation_Execute()   [CTRL] 读编码器 → 迭代修正           │
│        ↓                                                        │
│ Ctrl_ClosedLoop_SyncTarget()  [CTRL] PID 目标同步                  │
│        ↓                                                        │
│ printf("ok\r\n")              [BSP]  UART1 TX → 上位机确认         │
└──────────────────────────────────────────────────────────────────┘
```

**一个请求穿越了 4 层**, 每一层只做自己职责范围内的事, 通过接口调用下一层。

---

## 五、高解耦的四大技术手段

### 5.1 依赖注入 (DI)

```c
// ❌ 旧代码: 模块内部 extern 全局变量 (隐式依赖)
void Ctrl_Gripper_Init(void) {
    extern TIM_HandleTypeDef htim2;   // 藏在实现里, 调用者不知道
    BSP_Gripper_Init(..., &htim2, ...);
}

// ✅ 新代码: 参数注入 (显式依赖)
void Ctrl_Gripper_Init(TIM_HandleTypeDef *htim) {
    BSP_Gripper_Init(..., htim, ...);  // 调用者决定传哪个 Timer
}

// main.c 作为组装根完成注入:
Ctrl_Gripper_Init(&htim2);
```

**价值**: 换一个 Timer 引脚只需改 main.c 一行, 模块内部零改动。

### 5.2 头文件隔离 (Header Hiding)

```c
// ❌ 旧 ctrl_closed_loop.h: 暴露 BSP 类型
#include "bsp/bsp_as5600.h"
AS5600_Dev_t *Ctrl_ClosedLoop_GetEncoder(int axis);  // 泄露了 AS5600_Dev_t

// ✅ 新 ctrl_closed_loop.h: 只暴露语义接口
bool Ctrl_ClosedLoop_GetAxisAngle(int axis, float *out_deg);
ErrorCode_t Ctrl_ClosedLoop_SetAxisZero(int axis);
// 调用者不知道底层是 AS5600, 换一个编码器品牌只需改 .c 文件
```

**价值**: 如果某天编码器从 AS5600 换成 AS5048A (SPI 接口), CONTROL 层 `.c` 改完即可, 所有上层代码零改动。

### 5.3 薄包装 / 委托模式 (Delegate Pattern)

```
APP 层想开夹爪:
  Ctrl_Gripper_Open()           ← APP 调用这个
      ↓ (ctrl_gripper.c 转发)
  BSP_Gripper_Open(GetHandle())  ← 实际干活的

APP 层想读手柄:
  Ctrl_PS2_ReadData(&data)      ← APP 调用这个
      ↓ (ctrl_ps2.c 转发)
  BSP_PS2_ReadData(&data)       ← 实际干活的
```

**薄包装不是无意义的转发**——它在两层之间插入了一个**稳定的契约面**:
- 如果 PS2 手柄换成 USB HID 手柄, 只需改 `ctrl_ps2.c` 一行代码
- 如果舵机从 TIM2_CH2 换成 TIM3_CH1, 只需改 `Ctrl_Gripper_Init()` 的调用参数

### 5.4 自动机模式分离 (Mode-based Task Dispatch)

```c
if (mode == SYS_MODE_GCODE) {
    Task_EncoderRead();    // 仅在 G-code 模式读编码器 (PID 需要)
    Task_ClosedLoop();     // 仅在 G-code 模式运行 PID
}
// PS2 模式下跳过所有不必要的 I2C 读取, 保证实时性
```

**设计原理**: 不需要的功能不消耗 CPU。PS2 遥控时不需要编码器追踪 (用户直接控制), 因此不在主循环中阻塞 I2C。

---

## 六、模块职责矩阵

```
                      │ 硬件 │ 算法 │ 状态 │ 外部 │ 测试
模块                   │ 依赖 │ 逻辑 │ 管理 │ 接口 │ 难度
──────────────────────┼──────┼──────┼──────┼──────┼─────
app_teleop            │  无  │  无  │ 模式 │ PS2  │ 易
app_gcode_exec        │  无  │  无  │ 坐标 │ UART │ 易
app_gcode_parser      │  无  │  无  │  无  │  无  │ 极易
app_calibration       │  无  │  无  │  无  │  无  │ 易
──────────────────────┼──────┼──────┼──────┼──────┼─────
ctrl_planner          │  无  │ ★★★ │ ★★   │  无  │ 中
ctrl_kinematics       │  无  │ ★★★ │  无  │  无  │ 极易
ctrl_motion_engine    │ ISR  │ ★★★ │ ★★★  │  无  │ 难
ctrl_closed_loop      │  无* │ ★★★ │ ★★★  │  无  │ 中
ctrl_compensation     │  无* │ ★★  │ ★★   │  无  │ 中
ctrl_ps2              │  无  │  无  │  无  │  无  │ 极易
ctrl_gripper          │  无  │  无  │  无  │  无  │ 极易
──────────────────────┼──────┼──────┼──────┼──────┼─────
bsp_stepper           │ GPIO │  无  │  无  │  无  │ 中
bsp_as5600            │ I2C  │ ★   │ ★★★  │  无  │ 中
bsp_gripper           │ PWM  │  无  │ ★    │  无  │ 中
bsp_tmc2209           │ UART │  无  │  无  │  无  │ 中
bsp_ps2               │ SPI  │  无  │ ★    │  无  │ 中
bsp_homing            │ 限位 │ ★   │ ★    │  无  │ 难
bsp_uart1             │ UART │  无  │ ★    │  无  │ 中
bsp_led               │ GPIO │  无  │  无  │  无  │ 极易

* 通过 Ctrl_ClosedLoop_GetAxisAngle 间接访问, 不直接调 BSP
```

---

## 七、关键数据结构的"层归属"

```
结构体                │ 定义位置          │ 归属层  │ 可见范围
──────────────────────┼──────────────────┼────────┼────────────
GCodeFrame_t          │ app_gcode_parser.h │ APP    │ APP 层
SystemMode_t          │ app_teleop.h       │ APP    │ APP 层
RobotAngles_t         │ ctrl_kinematics.h  │ CONTROL│ CONTROL+APP
RobotMotorUnits_t     │ ctrl_kinematics.h  │ CONTROL│ CONTROL+APP
MotionFrame_t         │ ctrl_motion_engine │ CONTROL│ CONTROL+APP
PS2_Data_t            │ bsp_ps2.h          │ BSP    │ (通过 ctrl_ps2 转发)
AS5600_Dev_t          │ bsp_as5600.h       │ BSP    │ CONTROL 层内部
Stepper_Dev_t         │ bsp_stepper.h      │ BSP    │ CONTROL 层内部
Gripper_Dev_t         │ bsp_gripper.h      │ BSP    │ CONTROL 层内部
ErrorCode_t           │ error_code.h       │ COMMON │ 全局
```

**规则**: APP 层代码中不会出现 `AS5600_Dev_t*`、`Stepper_Dev_t*`、`Gripper_Dev_t*` 等 BSP 类型。APP 只知道 `float`、`int`、`bool` 和控制层定义的结构体。

---

## 八、从 B+ 到 A (85→95) 的优化历程

```
B+ (原始状态)
├── 5 条跨层违规: APP 直接调 BSP_Gripper_*, BSP_AS5600_*
├── CONTROL header 泄露 BSP 类型: AS5600_Dev_t*
├── 多余 HAL includes 在 app/control header
├── BSP 层风格不一致: stm32f4xx_hal.h vs main.h
└── Gripper 用 extern 全局变量

    ↓  第一轮优化 (层间解耦)

A− (92分)
├── 新建 ctrl_gripper 模块 → APP 不直接调 BSP_Gripper
├── 删 GetEncoder(), 加 GetAxisAngle/SetAxisZero → 头文件不泄露 BSP 类型
├── 清理 header HAL includes
├── BSP 6 个 header 统一用 main.h
└── Task_EncoderRead 仅在 G-code 模式运行 (PS2 模式免阻塞)

    ↓  第二轮优化 (深度解耦)

A (95分)
├── 新建 ctrl_ps2 模块 → APP 完全不 include BSP
├── ctrl_gripper 用依赖注入 → 删 extern htim2
└── ctrl_compensation 编码器访问走闭环层 → 全部 CONTROL 模块统一编码器路由
```

---

## 九、架构的"可教学性"

这套架构之所以适合做教学演示, 是因为每个模块的**学习难度分层**:

| 学习阶段 | 需要理解的模块 | 前置知识 |
|----------|---------------|----------|
| **入门** | COMMON (robot_config.h 调参) | 无 |
| **基础** | BSP 层逐个模块 (I2C/PWM/GPIO 驱动) | STM32 HAL 基础 |
| **进阶** | ctrl_kinematics (SCARA 逆运动学) | 三角函数, 连杆几何 |
| **进阶** | ctrl_closed_loop (PID 控制) | 控制理论入门 |
| **进阶** | ctrl_motion_engine (Bresenham ISR) | 中断, 定时器 |
| **高阶** | 四层解耦设计 (本文档) | 软件工程, 设计模式 |

**学生可以逐层深入**: 先改 `robot_config.h` 的参数看效果, 再逐模块阅读 `.c` 实现, 最后理解架构设计意图。

---

## 十、快速参考: 如果想新增功能...

| 需求 | 改什么 | 例子 |
|------|--------|------|
| 改 PID 参数 | `robot_config.h` | `CL_KP 0.8f → 1.2f` |
| 改连杆长度 | `robot_config.h` | `LINK_1_LEN 140.0f → 150.0f` |
| 改回零回退距离 | `robot_config.h` | `HOMING_BACKOFF_M1_DEG` |
| 加新的 G-code 指令 | `app_gcode_parser.c` + `app_gcode_exec.c` | `G28` (归零) |
| 换编码器型号 | `bsp_as5600.c` + `bsp_as5600.h` | AS5600 → AS5048A |
| 换电机驱动芯片 | `bsp_tmc2209.c` + `bsp_tmc2209.h` | TMC2209 → TMC5160 |
| 换手柄 | `ctrl_ps2.c` + `ctrl_ps2.h` (转发层) | PS2 → 蓝牙手柄 |
| 加传感器 | 新 `bsp_xxx.c` + 新 `ctrl_xxx.c` | 力传感器、陀螺仪 |
