# 正常启动与M999的动作前审计

软件103增加IRA1每轴独立先32/尾32附加存储，观察该轴IMR关闭后冻结；全局IRQ2/旧解析继续兼容。prepare_axis_capture.py从准确ELF生成只读配置，parse_axis_irq.py强制检查两头一致/偶数revision再解析。216项各、9ARM通过，未部署；当前硬件102为97锁定IRQ6，根因未定。见[软件103](../../docs/software_validation/20261021_axis_irq_evidence_103/README.md)。

当前为软件88／硬件89。共享限位分发已修正为“使能pending走HAL、屏蔽pending只清不回调”，故障屏蔽关闭自身边沿并排空，锁存不在线清除。87实机发现只过滤回调仍会出现残留pending空转，89补全后同类控制由17780个向量降为4；初始启动pending来源仍未定，见[最终只读验证](../../docs/hardware_validation/20261021_irq_drain_readonly_89/README.md)。IRQ2完整记录现在由显式向量退出钩子发布，不依赖M3分支被调用。以下83描述为历史基础。

软件83新增`TeachingStartupIrqAudit`：在永久抑制输出的前提下，用IRQ2保存共享入口以及三次真实HAL处理前后的pending/IMR/GPIO与回调轴，64条完整发布后冻结。M115增加IRQ_EVIDENCE能力，记录器不清标志、不吞回调、不修改安全预算。离线解析见`parse_irq_evidence.py`，实机自然窗口与逐轴SWIER正对照见[硬件84](../../docs/hardware_validation/20261021_irq_readonly_84/README.md)。正对照通过不代表原突发根因已解决。

预设 `TeachingStartupAudit` 使用真实AppRuntime、协议、Safety、Device/BSP、TMC确认与编码器读取。在三路驱动确认和同轮反馈检查通过后，进入 `STATE:OUTPUT_LOCKED HOMED:0 ALLOWED:0`，停在夹爪初始化/EN使能前；不模拟回零、标定或READY，不清首故障，不返回M999OK。正常构建默认关闭该模式，保留原功能。

```powershell
cmake --preset TeachingStartupAudit
cmake --build --preset TeachingStartupAudit
```

M115明确返回 `FIRMWARE:RobotArmStartupAudit PROTOCOL:AUDIT1 UART:115200 MOTOR_ENABLE:LOCKED AUTO_MOTION:0`，原正常固件仍为PROTOCOL:1。查询、M112、M999由真实生产协议处理；M999在此变体回复“startup checks started; outputs locked”，表示启动检查请求，不是回零完成。OUTPUT_LOCKED不接受动作，重复恢复请求按“进行中”拒绝；无故障时M999仍拒绝。M112将该边界置FAILED，后续M999重新配置/确认驱动和反馈，旧故障保留。

审计边界明确返回主循环，不执行READY周期任务；`startup_audit_tests`编译真实App/协议并包装READY服务计数，验证锁定状态不会落入任务调度。另有驱动失败、编码器超时、检查中停止及恢复回归。与正常构建的既有测试共同验证默认功能不被审计模式改变。

第二层保护位于 `guard.c`：链接器包装main，先禁止输出，再沿用Core时钟/GPIO/UART/I2C初始化，跳过TIM2/TIM6初始化；随后调用真实App_Init/App_RunOnce。GPIO写包装器拦截EN低、STEP/PA1高，另拦截STEP函数、PWM和定时器启动；SysTick重复禁止输出，PC0 LED常亮。保护不是回零替身，正常测试中blocked_outputs应为0，而非依靠拦截大量动作假装流程无动作。

`g_startup_audit`是独立审计记录，九个32位字段：magic、blocked_outputs、prepare_calls、prepare_real_ok、prepare_forced_fail、encoder_reads、encoder_actual_mask、inject_prepare_once、inject_encoder_mask。只能在本专用构建里通过SWD显式注入失败：一次准备失败会跳过本次实际TMC配置；编码器掩码仍先真实读取，只把有效结果强制标为失败。注入不制造有效反馈，也不代表实际硬件发生故障。每个ELF须重新解析符号地址并核对magic，禁止沿用旧地址盲写。

实机证据见 [阶段27](../../docs/hardware_validation/20261020_startup_recovery_audit_27/README.md)。输出约束与SWD快照不代替示波器测量；本次未测复位瞬态、真实运动、夹爪、相电流或机械精度。M3原始重复回调仍是独立未定问题。
