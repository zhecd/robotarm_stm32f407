# 最小GPIO/EXTI对照入口

此入口仅用于电机、夹爪断开时的软件定位，不提供运动功能。默认TeachingExtiProbe独立链接ST启动文件、CMSIS系统文件、main.c与probe_logic.c，不链接HAL、RobotArm业务库或原Core/main.c。PD2保持高电平禁用驱动，三路STEP保持低，TIM2/TIM6不初始化，UART/I2C/PS2均不配置。不发送串口命令，COM口无回复是默认入口的预期行为，不能误认为正常功能失效。

时钟保持与正常固件相同的168MHz HSI-PLL设置；主循环活跃采样，SysTick为1ms。按顺序执行四个20秒阶段后关闭限位检测并冻结统计：

| 阶段 | 检测方式 | 要回答的问题 |
|---|---|---|
| 0 OFF | 关闭上升/下降沿检测，1ms采GPIO；若出现pending则记录并按位清除 | 检测关闭时是否仍有pending，输入样本是否变化 |
| 1 POLL | 仅开下降沿，屏蔽NVIC/IMR，1ms读pending并清除 | 不进入EXTI处理函数时，是否仍会产生pending |
| 2 IRQ | 直接EXTI处理函数，按位写1清除并执行DSB后读回 | 移除HAL及业务后是否仍会进入，清除后读到什么 |
| 3 OFF_AFTER | 再次关闭检测并清pending，继续GPIO采样 | 检测关闭后的对照是否一致 |

IRQ阶段按轴10ms/256次预算屏蔽过密输入，以保证SysTick和只读取证能继续。它是本独立诊断的预算，不能借此改变正常/WAVE固件的过载恢复合同。阶段切换只发生在本诊断程序，无STEP输出能力。

`g_limit_probe` 保存版本、时钟、阶段、完成标志及各阶段统计；`g_limit_probe_trace` 保存128条52字节事件，满后冻结并计丢弃。MS/phase/PR前值、处理位图、PR后值、GPIO前后、IMR/FTSR/RTSR/EXTICR4和周期计数均入记录。寄存器并非同时采样。

轮询pending会合并采样间隔内的多次事件，不能直接与IRQ次数比较。1ms GPIO采样也可能漏掉更短脉冲。写清后仍pending，既可能是未清除，也可能是新的事件；仅凭这一值不能定因。四阶段无事件也不等于故障修复。移除其他模块同时改变了软件负载和外设活动，属于缩小范围的对照，不是对全部根因的排他证明。

```powershell
cmake --preset TeachingExtiProbe
cmake --build --preset TeachingExtiProbe
cmake --build --preset HostTests --target limit_probe_logic_tests
ctest --test-dir build/HostTests -R '^limit_probe_logic_tests$' --output-on-failure
```

电脑测试编译真实统计源码，验证GPIO变化、pending/IRQ区别、独立预算、时差回绕和无pending入口。不模拟真实W1C行为或外部电气信号。实际Flash回读、运行和RAM结果必须另存硬件证据，不能从编译或电脑测试推导。

首次实测完成后使用WFI等待时，运行态调试连续读取头部全零，暂停后读回完整原数据，4阶段均有20000个样本。这是本次已观察到的读取限制，不能据此宣称RAM内容丢失或程序重启。后续源码完成后改为活跃等待，避免该场景；首次失败与暂停取证保留在 `docs/hardware_validation/20261019_200850_minimal_exti_probe`，不得覆盖为全程成功。

清pending方式与工程内HAL宏及 [ST RM0090](https://www.st.com/resource/en/reference_manual/rm0090-stm32f407-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) 一致，按目标位写1；不对PR做读改写，也不使用SWIER制造测试事件。没有根据未确认的芯片或电气原因加入补丁。

## UART轮询对照

`TeachingExtiProbeUart` 在相同GPIO/EXTI程序上增加USART1寄存器轮询收发、纯回复器与已有纯C行组帧器。仍不链接HAL、DMA、UART中断、编码器和业务服务。PA9/PA10的AF7、无上下拉、VERY_HIGH输出速度与原UART1配置对齐，波特率115200；这是对照条件，不预先把该速度认定为正确或错误。

仅接受M115/M408/M114/M119，标识为LimitProbe/LPUART1；M408的PENDING是当前阶段观察次数，阶段变化后不是启动以来累计总数，M114明确VALID:0,0,0且ENCODER_POLL:0。纯回复生成有独立电脑测试，不把拒绝的动作映射到任何设备调用。

`run_board.py --run-authorized-isolated-test --probe-preset TeachingExtiProbeUart --current-waveform-baseline --inject-exti-control` 在已有现场授权、执行器断开且板上Flash与已验证波形固件一致时执行，完成后独立回读并恢复该波形固件。UART查询子进程仅发送四种查询，约75秒结束。若选择控制注入，主机在IRQ阶段约第17秒记录自然观察前缀，再向SWIER注入一次明确标识的EXTI12事件验证链路；该尾部是控制窗口，不能混入自然事件结论。固件自身不写SWIER。实际使用须保留完整stage_record、uart_capture、probe_result及源报告哈希。
