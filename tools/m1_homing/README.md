# 公共单轴低速验证（历史M1入口）

软件阶段54新增`axis_home_contract.ps1`和`execute_axis_home_after_approval.ps1`：纯回复合同与实际通信调度分离，缺少已确认机械方向及依据时在打开串口前拒绝。自检三轴两向/8限位掩码/71非法案例，Debug/Release各111项通过，尚未实机HOME。用户当前已断电；下午重新读取状态后继续。两文件须一起复制到新验证目录，不复用旧claim，具体合同见[阶段54](../../docs/software_validation/20261021_axis_home_supervisor_54/README.md)。

最新实测为阶段52 M3 AXISHOME1已用会话，三个电机按用户更新授权均连接；仅M3发20微步，结束共用EN禁用。M1/M2 STEP维持低电平，未采其编码器，不声称实体完全不动。阶段53新增纯离线[jog_trace_report.py](jog_trace_report.py)，Debug/Release各109项通过；分别报告有限动作合同与运动反馈，不升级机械方向、精度或HOME验收。以下51及更早为历史。

当前实机为[阶段51 M2](../../docs/hardware_validation/20261021_m2_first_jog_51/README.md)：AXISHOME1/AXIS:2，锁定版20轮检查及一次20微步D0点动完成，输出禁用、会话已使用；参考仍UNSET。反馈有变化但机械方向和精度未验收。M3尚未部署，换轴前必须断电调整接线。以下M1记录按历史阶段理解，公共模块说明以[选轴准备](AXIS_PREPARATION.md)为准。

当前软件新增axis_binding和SINGLE_AXIS_ID=1/2/3，三轴复用同一套状态机、局部参考、接收和截止模块。M1保持M1HOME8；M2/M3使用AXISHOME1并报告所选轴。默认构建禁止动作。用法、板端映射和测试顺序以[AXIS_PREPARATION.md](AXIS_PREPARATION.md)为入口。

软件阶段49/50已完成Debug/Release各108项、三轴锁定/动作共6种ARM构建和主机合同检查，均未烧录。硬件最后实测仍阶段48的M1HOME8已用会话、驱动禁用。下一项M2实测必须先断电换接电机；下文48及更早均为历史。

2026-10-21阶段48当前现场：用户许可一次M1HOME8 HOME并要求加快流程、不依赖人工实时反馈。已部署动作候选1510babef400b60b9b5cc8330d8b5b483fd0ee9f7ed3e07f1027daf901bb30ab，49512字节独立回读匹配；一次HOME寻35/退40，50微步/s、200寻程/40退回/5500ms未变。DONE5/REASON0/USED1/EN禁用；M122共30帧，UNSET0→VALID2实测，origin291.006，停机后约3秒有效/相对关节0；HOMED/ALLOWED仍0。99反馈点最大间隔93ms/年龄19ms；M1 LOW468/变化2，M2M3为0，最终三路OPEN。SWD EN高/三轴STEP及PA1低/TIM5停/CFSR HFSR0/主循环推进，LED ODR0xE，CPU运行，COM4/SWD关闭，115200/LOW。当前MOTION1已用会话，不是编译锁定；Flash SHA9b6468c385b4818eee05d3c26301acb1ecaf985b33d49c60eb25b81a12d56779。M1 CN15，其余电机夹爪断开、ST-Link3V3不接。仅主机脚本加连续M122日志，有备份、自检和本次实测；固件沿用46的90项各/两ARM，未重复重跑。此次只用自动记录判断，无人工目视确认；后续不重复索要每轮目视反馈，合并检查归档，不把催促当作提高电机速度。新动作/接线调整仍按明确授权范围，不复位清USED重放，不自动重试。全局机械零位/重复精度及原M3 EXTI根因仍未定。见docs/hardware_validation/20261021_m1_reference_home_once_48/README.md。

2026-10-21阶段47为当前现场：按用户明确许可仅部署M1HOME8编译锁定版（MOTION0/AUTO0），48840加载字节独立回读匹配，ELF SHA6c651b3c0b34c75c197213ae0167ac63c09448d440a62de0a3b5293274473130，Flash SHA554f0e135deb3b1aab4555a6bacbb1143131f70f6dd1dd1ace9476c655c4df27。30+200轮只读通过，M122 LOCAL_REF0/VALID0/REASON1是尚未HOME的预期状态；WHOLE_HOMED/ALLOWED0。IDLE/USED0/EN禁用/动作计数0，三路OPEN、LOW/CHANGES均0；编码器有效290.953..291.006、最大AGE19ms，115200/LOW。仅M115/M408/M114/M119/M120/M121/M122，无CONFIG/DIR/HOME/JOG/M112。SWD末态EN高、三轴STEP/PA1低、TIM5停、CFSR/HFSR0、主循环推进，LED ODR0xE，CPU运行，COM4/SWD关闭。一次SRAM检查误按32位枚举，核ELF DWARF为1字节后仅修正脚本并通过，原误报留档；今后SRAM须核当前ELF宽度/偏移，不能猜布局。M1 CN15，其余电机/夹爪断开，ST-Link3V3不接。新局部参考VALID转移尚未实测，原M3 EXTI根因未定。90项各与两ARM沿用46，无源码变化。动作8仅build/M1ReferenceMotion_46（SHA1510babef400b60b9b5cc8330d8b5b483fd0ee9f7ed3e07f1027daf901bb30ab），未烧录；新动作须另授权，保留旧claim。见docs/hardware_validation/20261021_m1_reference_locked_47/README.md。

阶段46仅完成软件：新增无HAL的`m1_reference`及只读M122，M1HOME8在HOME成功退回后等待新编码器样本，建立本次启动的局部参考点；不会设置整机HOMED或开放动作。反馈无效/超时、限位或状态异常时失效锁存，不自动重建。Debug/Release各90项及两ARM构建通过，未烧录、未复位、未打开串口；现场仍为下面阶段45已用会话。模块依赖、中文说明和测试顺序见[ARCHITECTURE.md](ARCHITECTURE.md)。

2026-10-21当前为阶段45：M1HOME7带LED及1ms输入历史，阶段44首次HOME寻99/退40成功；用户另授权“再来一次”，阶段45核Flash相同、仅复位一次，HOME寻40/退40成功。各仅一次HOME，无自动重试，50微步/s、200步最大寻程、40步退回、5500ms边界未改。末态DONE5/REASON0/USED1/EN禁用/STEP0/HOMED0/ALLOWED0；COM4/115200/LOW，三路OPEN，LED1亮、2/3/4灭。45记录104反馈点、最大间隔78ms/年龄17ms，M1 LOW479/变化2、M2M3均0；禁用后约3秒291.006稳定。EN高/其他STEP与PA1低/TIM5停/CFSR HFSR0/主循环推进，CPU运行，串口和SWD关闭。动作镜像SHA20424921629cd7044921a3eeab0281a851827b47bcf2791ebe8f01f3ee3509ee，Flash SHA8484c765dc68d32f448a023b2a4214121b7923aaf0dda989a95996c74965b6f6。M1 CN15，其余电机夹爪断开、ST-Link3V3不接。不是编译锁定，不是整机建立零位；用户已确认第二次全程看清、靠近/触发亮灯/反向退回/熄灭均正常，前两次未触发和M3 EXTI根因未定。82项各及两ARM沿用43，无生产源码变化。保留44/45全部claim，无新动作授权，不复位清USED重放。详见docs/hardware_validation/20261021_m1_led_home_repeat_45/README.md。

最新为[阶段43](../../docs/hardware_validation/20261021_m1_limit_led_trace_43/README.md)：M1HOME7编译锁定已部署。LED1/PC0常亮，LED2/3/4分别实时显示M1/2/3，和HOME使用同一1ms快照；用户手按及1819轮查询确认映射。M120读取IDR/模式/上拉/LED映射，M121读取低采样/变化/曾低掩码；仅记录轮询已采到的电平，不替代10ms资格确认或捕获所有亚毫秒脉冲。82项Debug/Release、两ARM通过。当前不允许运动，新HOME须单独确认；保留全部旧claim，用户已撤销硬件电压测量。下方40/41等均为历史。

最新[阶段41实机](../../docs/hardware_validation/20261021_m1_home_once_41/README.md)：一次HOME走到SEEK200仍未触发，TRAVEL_BOUND后M112收尾，现STOPPED/USED1/EN禁用。没有D1退回或追加搜索；未完成回零。用户未看清是否接触触片，下一步先确认几何位置。M1HOME6已部署，下面阶段40“未部署/待确认”为当时的历史准备记录；不得复用本次动作凭据或自动放大行程。

[阶段40寻限位准备](../../docs/software_validation/20261020_m1_home_preparation_40/README.md)已完成：M1HOME6显式报告200步寻限位/40步退回/5500ms总限/20ms步周期。默认拒绝的专用HOME主机脚本、两向末端触发和5500个中止偏移测试已加入；Debug/Release各81项及两种ARM构建通过。10轮旧镜像无动作复查通过，候选未烧录、未复位或动作。正式动作须确认允许D0寻限位后D1退回；最后实测方向D0靠近限位，D1未单独验收。

最新实测为[阶段39](../../docs/hardware_validation/20261020_m1_joint10_repeat_39/README.md)：两次400步D0均分别获授权并完成，末次反馈换算关节约10.02°、禁用后3秒稳定，用户确认D0靠近M1限位。当前专用400步/9秒镜像DONE/USED1，驱动禁用，串口115200，三路限位OPEN；没有回零，未经新许可不追加动作或复位。以下37为准备历史，36及更早为历史实测。

[阶段37电脑准备](../../docs/software_validation/20261020_m1_joint10_preparation_37/README.md)新增关节10°专用配置：`M1_JOG_JOINT_TEN_DEG=ON`选择400微步/9000ms，仍50微步/s，与电机轴10°选项互斥。TIM5允许该9秒点动，HOME业务截止仍5500ms。Debug/Release各80项及专用锁定/动作两种ARM构建通过；该阶段均未部署；后续按独立许可完成38/39。M115仍M1HOME5，但该专用配置会报告JOG_STEPS:400/JOG_DEADLINE_MS:9000，主机必须精确核对。下面默认20和历史89参数按对应配置解释。

历史[阶段36](../../docs/hardware_validation/20261020_m1_repeat_continuous_36/README.md)：用户再次明确批准同参数，独立复位一次后执行89步D0一次，110点连续反馈，电机轴原始差+8.754°、禁用后3秒稳定；按4.5配置折算关节约1.945°。用户反馈转动太小，实际方向仍待确认，不追加动作；当前为M1HOME5动作编译开启但已用会话，复位后重试必须有新授权。下面35为历史。

历史[阶段35](../../docs/hardware_validation/20261020_m1_motor10_jog_35/README.md)：M1HOME5报告实际点动步数/截止。默认20步/500ms不变，显式`M1_JOG_MOTOR_TEN_DEG=ON`选择89步/2000ms，仅允许独立M1入口。获本次许可执行一次89步D0，终态DONE/USED1/EN禁用；用户未确认明显转动，实际角度/方向尚未验收，用户确认期间手动调整过，后续反馈不用于本次位移验收。当前板上MOTION_COMPILED1的已使用会话，不是编译锁定版；不得复位后重放。Debug/Release各78项通过。阶段34的UG/NVIC修复保留，GUARDTEST只在编译锁定下装载、不启动计数。以下阶段31和默认参数均应结合此处显式配置解释。

[阶段31内部电流参考锁定版](../../docs/hardware_validation/20261020_m1_internal_current_31/README.md)为历史：独立M1显式选择内部参考、继续使用外部R110，不要求调整电位器或测VREF；该电流合同在后续版本保留。正常固件兼容入口仍为VREF模式。阶段29已完成串口缓存及锁定状态下TIM5中断路径/IWDG复位/M112验证；它们不等于5.5秒实际运动截止或物理脉宽验收，也不证明M3原始EXTI根因修复。

本模块不替换正常三轴回零。独立入口不链接App、普通运动规划、三轴Homing或夹爪；不建立整机零位，不清正常Safety故障。M2/M3 STEP和夹爪PA1始终低，TIM2/TIM6不初始化。共用EN仍同时连接三颗驱动，所以**M2/M3电机及夹爪必须继续物理断开**。

默认预设 `TeachingM1HomingPrepare` 明确设置 `M1_HOME_MOTION_ENABLED=OFF`。纯状态机Start和最终GPIO Apply各检查一次编译许可，即便输入JOG/HOME也返回输出锁拒绝。上电不自动配置TMC、不使能、不点动或回零。后续动作版本必须另行明确编译许可，并获得用户对该次速度、方向、行程的确认。另一个启用分支仅做交叉编译检查，**不放入交付固件清单**，不构成动作授权。

```powershell
cmake --preset TeachingM1HomingPrepare
cmake --build --preset TeachingM1HomingPrepare
cmake --build --preset HostTests --target m1_homing_logic_tests
ctest --test-dir build/HostTests -R '^m1_homing_logic_tests$' --output-on-failure
```

## 模块与调用合同

`m1_logic.h/.c` 只接收限位、反馈有效位、时间和许可，输出M1的DIR/STEP/EN意图，完全无HAL依赖。`main.c`绑定真实USART1/6、TMC确认、M1编码器和GPIO，负责传感器新鲜度、硬截止与输出。SysTick每1ms调用状态机，单次固定工作量，不在ISR读取I2C或打印；主循环每20ms真实读取M1反馈。跨ISR状态读取/发布使用短临界区。

状态编号：IDLE0、JOG1、SEEK2、QUALIFY3、BACKOFF4、DONE5、STOPPED6、FAILED7。原因：OK0、HOST_STOP1、TRAVEL_BOUND2、TIME_BOUND3、LIMIT_UNSTABLE4、LIMIT_STUCK5、OTHER_LIMIT6、FEEDBACK_LOST7、HOST_LOST8、MAIN_STALLED9、LIMIT_DURING_JOG10、INPUT_FAULT11。DONE只是本单轴限位/退回测试结束，不等于整机已回零；M408始终HOMED0/ALLOWED0。

## 待现场确认的动作参数

| 项目 | 固定边界 |
|---|---|
| STEP速率 | 50微步/秒，周期20ms，高电平设计1ms |
| DIR及EN建立 | 首脉冲前至少2ms；反向后重新等待2ms |
| 默认方向点动 | 20微步；总时限500ms；显式电机10°试验配置为89微步/2000ms，均需当次许可 |
| 寻限位 | 最多200微步，首次原始触发立即停寻限位脉冲 |
| 触发确认 | 低电平稳定10ms才允许反向；此前释放则失败，不反复往前试 |
| 反向退回 | 固定40微步，要求开关稳定释放10ms；未释放则失败，不追加行程 |
| 回零总时限 | 5500ms，TIM5独立硬截止；不补发迟到脉冲 |
| 主机租约 | 每100ms发KEEP；500ms未续约停止 |
| 主循环/反馈 | 主循环250ms未更新或反馈超过100ms不新鲜即停止 |
| 动作次数 | 每次MCU启动只允许一个动作，结束/停止后不能重放、M999重试或自动续发 |

按当前200整步×16细分、减速比4.5配置折算：点动约0.5关节度，最多寻限位5度，退回约1度，速度约1.25关节度/秒。数值不是已测位移/速度；实际细分、传动、脉宽、方向及停止距离尚未实机验收。若起点离限位超过5度，本试验会失败停止，不做全行程搜索。

实际动作前再次调用真实 `Drv_TMC2209_ConfigNodeVerifiedWithReference(...,TMC_CURRENT_INTERNAL_REF,...)` 确认M1配置，再读取新鲜反馈。准备测试采用IRUN编码8/IHOLD编码4、GCONF=0x1C0（bit0模拟参考关闭、bit1内部采样关闭）、CHOPCONF=0x14000053（vsense=0）。按模块照片R110=0.11欧和手册典型325mV，标称RMS运行/保持电流约0.497/0.276A，**这是计算设定，不是实测绕组电流，也不保证负载转矩足够**。该配置与正常默认28/15及VREF模式分开。初始M1限位须松开，其余限位不能触发。HOME必须先记录经现场确认的寻限位方向；方向点动结果需人工确认，不能根据DIR高低推断真实转向。

## 串口合同

USART1保持115200/8N1/GPIO LOW；CR/LF/CRLF结束，最多39个可打印ASCII字符。严格单请求、单回复，不排队。驱动准备期间收到当前行尾以外的新数据或发生UART错误，会拒绝启动并停止锁存，避免把已到达的停止请求延后到使能之后处理。

| 指令 | 行为 |
|---|---|
| M115 | `FIRMWARE:M1HomingPrepare PROTOCOL:M1HOME6 UART:115200 MOTION_COMPILED:0 AUTO_MOTION:0 JOG_STEPS:20 JOG_DEADLINE_MS:500 SEEK_STEPS:200 BACKOFF_STEPS:40 HOME_DEADLINE_MS:5500 STEP_PERIOD_MS:20`；显式点动配置可为89/2000或400/9000，动作编译许可另报告1 |
| M408 | 状态、原因、会话是否使用、实际EN/STEP引脚、各阶段软件计数；始终HOMED0/ALLOWED0；RXERR/RXOVF为UART错误/缓存溢出累计 |
| M119 | 三路限位低有效快照 |
| M114 | M1原始电机轴角、VALID和采样年龄；不是回零后关节角 |
| CONFIG | EN禁用下确认M1的16细分/8/4/内部参考配置；报告本次GCONF、CHOPCONF真实读回及IFCNT前后值，不启用执行器 |
| GUARDTEST | 仅编译锁定、空闲未用会话时允许；检查所选点动500/2000/9000ms装载后NVIC/UIF均清，无计数、不使能、不改变活动状态 |
| M112 | 停止并永久消耗本次启动的动作会话，查询仍可用 |
| DIR D0 / DIR D1 | 记录操作者确认的寻限位方向；无脉冲，已使用会话不可改 |
| JOG D0 / JOG D1 | 按M115报告的固定20/89/400微步方向检查；准备构建总是拒绝 |
| HOME | 固定边界的M1寻限位/退回；准备构建总是拒绝 |
| KEEP | 只续约仍有效的活动会话，不启动、不解锁、不能恢复过期动作 |

其他命令拒绝；活动期间错误命令、UART格式/溢出、失联、其他限位、反馈无效均停止。未打开EXTI，只按1ms轮询GPIO；这是单轴试验隔离方式，不证明M3原始中断问题被修好。IWDG仅在主循环和SysTick均推进时喂狗，配置典型约500ms；实际LSI公差/停机时序仍待在锁定构建上验收。M112依赖串口响应，现场仍须能够切断12V。

## 电脑验证与待实测项

`m1_homing_logic_tests`编译本模块真实状态机，覆盖两向精确脉冲、时钟回绕、限位资格暂停、固定退回、未碰限位、触发抖回、退回不释放、其他限位、反馈失效、主机失联、主循环超时、总期限、禁止补步和重放，另逐一检查500个停止时刻之后不再发脉冲。HAL/引脚/定时器并未被电脑模型验收。

当前接线与实测见 [阶段31记录](../../docs/hardware_validation/20261020_m1_internal_current_31/README.md)。`m1_homing_rx_tests`编译真实缓存模块，覆盖1000轮回绕、延后消费、UART错误、缓存满及预检期间新输入拒绝。TMC总线模型增加18432组参考源确认及模式位错误/非法枚举/掉包/兼容入口复原测试。用户明确暂不调整硬件、不测VREF；保留计算值与实测值的区别，正式动作仍需确认可动余量及现场停止条件。首次只拟方向点动，不能把一次点动批准扩展成随后自动回零。
