# 夹爪独立验证入口

此入口用于教学和单次MG90S检查，默认不输出PWM。它与正常App互斥链接，三轴EN始终为高、三个STEP始终为低。原上位机115200/8N1及原夹爪100°开、72°关、600ms参数保留；这里使用GRIP1专用命令，不能直接作为正常G-code固件使用。

## 按依赖顺序阅读

| 顺序 | 模块 | 职责与测试 |
|---|---|---|
| 1 | `gripper_logic.h/.c` | 无HAL的单次窗口状态机，拥有目标、USED、首次失败原因、主机租约和限位初态。PC测试注入时间、输入、失败；DONE只表示输出窗口结束。 |
| 2 | `../m1_homing/m1_limits.c`、`m1_rx.c`、`m1_deadline.c` | 复用限位快照/LED映射、限长接收队列、TIM5截止。m1是历史目录名，不代表本入口控制M1。 |
| 3 | `RobotArm/BSP/driver/src/drv_servo.c` | 真实PWM驱动。Bind仅绑定，不调用正常Init的自动闭合；预载CCR后启动，HAL失败不报告目标完成。另有独立驱动测试。 |
| 4 | `main.c` | 组合GPIO、SysTick、UART、TIM2、TIM5、看门狗及逻辑状态；拥有硬件停止路径。EN/STEP没有动作入口。 |
| 5 | `gripper_contract.ps1` | 严格解析身份、状态、目标和限位；SelfTest不打开串口。 |
| 6 | `query_locked.ps1` / `execute_after_approval.ps1` | 前者只读默认锁定版；后者经明确动作许可、精确身份及一次claim后，仅发送一次OPEN或CLOSE，续租并监督结束。 |

课程可以按表中依赖顺序安排：纯逻辑→传感器与缓冲→PWM端口→嵌入式组合→上位机契约→实机取证。PC通过与实机通过分别记录。

## 构建和默认行为

`TeachingGripperPrepare`固定`GRIPPER_TEST_MOTION_ENABLED=OFF`。该ELF经符号检查不含HAL_TIM_PWM_Start和Drv_Servo_TrySetAngle，TIM2不初始化、PA1保持低。动作候选必须另行显式配置ON，启动后仍保持IDLE/PWM关闭，收到一个目标才启动。

动作候选输出50Hz，OPEN=100°/CCR1611µs、CLOSE=72°/CCR1300µs，最大600ms。TIM5独立截止先切PWM，SysTick再结束逻辑状态；主循环停滞200ms、主机租约250ms过期、串口错误、输入异常、限位变化均停止。IWDG为补充保护，其实际时长依赖LSI，未作实测精度承诺。主机监督阈值800ms，正常约70ms发KEEP；任何串口调用还有其本地超时，物理600ms边界由固件负责。

限位可以在动作前静态触发，因为此独立入口永久禁止关节输出；动作窗口内任何一路限位快照变化均失败停止。此规则没有改动正常App的限位保护，不把M2触发视为整机就绪。LED1为工作灯，LED2/3/4对应M1/2/3限位。

## 命令与重放保护

- M115：身份、编译许可、角度及窗口；M408：状态/首因/USED/PWM/目标/限位/串口错误；M119：限位。
- OPEN或CLOSE：每次启动只接受一个目标；无自动相反动作，无任意角度，无回零/步进命令。
- KEEP：只续250ms租约，不能延长600ms窗口；M112：立即停止。
- 未知/过长/非ASCII命令：停止。DONE/STOPPED/FAILED不允许再启动，固件没有清USED命令。

主机执行器缺少ExecuteAuthorized、目标或独立RecordDirectory时，在构造串口前拒绝。目录已有claim/result/trace时拒绝重放；身份不符不发送M112，身份符合后失败保留首因并停止，不复位、不重试。复位会清RAM的USED，但不会删除主机claim，仍须新动作许可。

串口查询仅证明命令/寄存器状态，MG90S无位置反馈，不能证明夹爪实际到100°/72°、夹持力或动作精度。首次实机动作需明确许可；保留原参数不等于已验收机械行程。
