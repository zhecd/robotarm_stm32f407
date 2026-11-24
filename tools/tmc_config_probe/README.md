# EN锁定的TMC配置确认诊断

`TeachingTmcConfigProbe` 是独立Flash入口，编译真实生产TMC驱动、BSP三节点准备及真实ST HAL/USART配置。它不链接正常App、回零、夹爪、STEP脉冲函数或定时器启动，没有机械动作命令。上电不自动写TMC；配置确认须显式请求。不是旧BENCH3电机台架，不接受其动作指令。

```powershell
cmake --preset TeachingTmcConfigProbe
cmake --build --preset TeachingTmcConfigProbe
```

串口为USART1 115200/8N1、GPIO LOW。接收CR/LF/CRLF，完整行最多39字符；只接受下表精确语法，其余拒绝。一次处理一个请求，主机等待回复后再发下一条；配置有界阻塞但无自动重试。先用M115确认 `FIRMWARE:TmcConfigProbe PROTOCOL:CFG1`。

| 指令 | 行为 |
|---|---|
| M115 | 固件身份、115200、输出锁定、无自动动作 |
| M408 | EN输入电平、STEP输入位图、TIM2时钟、违规输出请求数、确认次数、运行毫秒 |
| M122 N0..N3 | 只读IOIN/IFCNT/GCONF/CHOPCONF；VALID/ERROR/REG标识失败 |
| M780 N0..N3 | 调用真实ConfigNodeVerified，返回结果、读错误、失败寄存器、IFCNT前后与最后比较值 |
| M781 | 调用真实BSP_RobotBoard_PrepareMotionDrivers，依次确认节点0/1/2，首错停止 |

M780使用当前正常BSP默认配置：16微步、IRUN28/IHOLD15、SENDDELAY2、模拟VREF。电流值是档位编码，不是实测安培。结果枚举见生产 `drv_tmc2209.h`；M780成功的EXPECTED/OBSERVED是最后核对的CHOPCONF掩码值，不是本次写入次数。接受次数由BEFORE/AFTER模256差计算，必须为4。

EN在设为输出前预置高，三轴STEP与PA1固定低；SysTick每毫秒重申禁止输出。HAL_GPIO_WritePin链接包装器阻止拉低EN或拉高STEP/PA1，VIOLATIONS记录被阻止请求，正常应为0。PA1为普通低电平输出，不初始化TIM2/PWM；无TIM6启动。PC0低有效LED常亮。

输出检查为程序约束、串口快照与SWD寄存器读回，不代替示波器对复位瞬态的测量；本次没有示波器采集。诊断入口不采集编码器/限位，也没有解决M3原始重复回调；正常固件的App启动/恢复仍需独立现场验收。

本次实机记录见 [阶段26](../../docs/hardware_validation/20261020_verified_tmc_config_26/README.md)。Flash写入与复位必须遵守用户当前授权；构建本身不连接设备。
