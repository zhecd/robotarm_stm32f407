/**
 * @file main.c
 * @brief 公共单轴低速试验入口；默认M1，准备构建编译锁住EN。
 * @details TIM5硬截止、SysTick状态机与主机租约、主循环心跳、IWDG共同收尾。
 * EN为三节点共用信号；允许动作时只接所选轴电机，其余电机与夹爪物理断开。
 */
#include "main.h"
#include "usart.h"
#include "i2c.h"
#include "device/dev_joint.h"
#include "driver/drv_tmc2209.h"
#include "platform_time.h"
#include "m1_logic.h"
#include "m1_rx.h"
#include "m1_deadline.h"
#include "m1_limits.h"
#include "m1_reference.h"
#include "robot_config.h"
#include "axis_binding.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#ifndef M1_HOME_MOTION_ENABLED
#define M1_HOME_MOTION_ENABLED 0
#endif
#if M1_HOME_MOTION_ENABLED != 0 && M1_HOME_MOTION_ENABLED != 1
#error Invalid M1_HOME_MOTION_ENABLED
#endif
#if M3_RETRY_500 && SINGLE_AXIS_ID != 3
#error M3 retry profile is restricted to axis3
#endif

static M1Test test;
static volatile M1Rx rx;
static volatile uint32_t main_seen,feedback_seen;
static volatile bool feedback_valid,deadline_trip;
static float motor_deg;
static bool driver_ready;
static volatile M1LimitTrace limit_trace;
/* 只由主循环读写；中断中的运动状态通过Snapshot复制，不在ISR做浮点坐标运算。 */
static M1Reference reference;
static const AxisBinding *axis;
static M1Test Snapshot(void);
/* 描述表保持无HAL以便主机测试；此处编译核对当前Core符号的位号。 */
_Static_assert(M1_STEP_Pin==(1U<<6) && M2_STEP_Pin==(1U<<7) && M3_STEP_Pin==1U,"STEP map changed");
_Static_assert(M1_DIR_Pin==(1U<<3) && M2_DIR_Pin==(1U<<4) && M3_DIR_Pin==(1U<<5),"DIR map changed");
_Static_assert(M1_STOP_Pin==(1U<<10) && M2_STOP_Pin==(1U<<11) && M3_STOP_Pin==(1U<<12),"Limit map changed");
static void Disable(void)
{
    GPIOD->BSRR=GPIO_PIN_2;
    GPIOA->BSRR=(GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7)<<16;
    GPIOB->BSRR=GPIO_PIN_0<<16;__DSB();
}
static void SafeGPIO(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_GPIOD_CLK_ENABLE();Disable();
    GPIO_InitTypeDef g={.Mode=GPIO_MODE_OUTPUT_PP,.Pull=GPIO_NOPULL,.Speed=GPIO_SPEED_FREQ_LOW};
    GPIOD->BSRR=GPIO_PIN_2|((GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5)<<16);
    g.Pin=GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;HAL_GPIO_Init(GPIOD,&g);
    g.Pin=GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7;HAL_GPIO_Init(GPIOA,&g);
    g.Pin=GPIO_PIN_0;HAL_GPIO_Init(GPIOB,&g);
    /* 板上LED1常亮；LED2/3/4分别对应M1/M2/M3，先预载灭灯电平再设输出。 */
    GPIOC->BSRR=(GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3)|(GPIO_PIN_0<<16);
    g.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;HAL_GPIO_Init(GPIOC,&g);
    /* 只读轮询三路限位，不开启EXTI；不把此隔离策略当作M3根因修复。 */
    g.Pin=GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;g.Mode=GPIO_MODE_INPUT;g.Pull=GPIO_PULLUP;HAL_GPIO_Init(GPIOC,&g);
}
static M1LimitSnapshot ReadLimits(void)
{ return M1Limits_Decode(GPIOC->IDR,GPIOC->MODER,GPIOC->PUPDR); }
static M1Inputs Inputs(uint32_t now,uint8_t active_mask)
{
    return AxisBinding_Inputs(SINGLE_AXIS_ID,active_mask,
        feedback_valid && (uint32_t)(now-feedback_seen)<M1_FEEDBACK_STALE_MS,main_seen);
}
static void Apply(void)
{
    /* 最后的硬件准许仍由编译常量控制；普通准备镜像不存在有效使能路径。 */
    AxisOutputs out=AxisBinding_Outputs(SINGLE_AXIS_ID,&test,M1_HOME_MOTION_ENABLED,deadline_trip);
    GPIOD->BSRR=out.d_bsrr;GPIOA->BSRR=out.a_bsrr;GPIOB->BSRR=out.b_bsrr;
    __DSB();
}
static void Stop(M1Reason reason)
{
    uint32_t key=__get_PRIMASK();__disable_irq();
    Disable();TIM5->CR1=0;M1Test_Stop(&test,reason);__set_PRIMASK(key);
}
void USART1_IRQHandler(void)
{
    /* SR后读DR清除RXNE/ORE等标志。只收字节，不在ISR打印或访问I2C。
     * 单字节轮询会在编码器阻塞期间溢出；128字节缓存承接主循环暂停。
     * 缓存满/硬件错误立即禁用输出并停止，不把残缺请求当成新指令。 */
    uint32_t sr=USART1->SR;
    if(sr&(USART_SR_RXNE|USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE)) {
        uint8_t ch=(uint8_t)USART1->DR;
        if(sr&(USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE)) {
            M1Rx_Error(&rx);Stop(M1_INPUT_FAULT);
        } else if(!M1Rx_Push(&rx,ch)) Stop(M1_INPUT_FAULT);
    }
}
void SysTick_Handler(void)
{
    HAL_IncTick();PlatformTime_OnTickFromISR();
    /* 同时保护状态与引脚收尾，避免TIM5停机后被抢占的代码重新拉高STEP。 */
    uint32_t key=__get_PRIMASK();__disable_irq();
    M1LimitSnapshot limits=ReadLimits();
    /* 同一1ms快照供历史、指示灯和HOME使用；串口查询间隙的低电平也留下记录。 */
    M1Limits_Observe(&limit_trace,limits.active_mask);
    GPIOC->BSRR=M1Limits_LedBsrr(limits.active_mask);
    uint32_t now=HAL_GetTick();M1Inputs in=Inputs(now,limits.active_mask);
    if (deadline_trip) M1Test_Stop(&test,M1_TIME_BOUND);
    else if (M1Test_Active(&test) && !limits.config_ok) M1Test_Stop(&test,M1_INPUT_FAULT);
    else M1Test_Tick(&test,now,&in);
    Apply();
    if (!M1Test_Active(&test)) TIM5->CR1=0;
    __set_PRIMASK(key);
}
void TIM5_IRQHandler(void) { if(M1Deadline_ExpiredFromISR()){deadline_trip=true;Disable();} }
void HardFault_Handler(void) { Disable();while (1) {} }
void MemManage_Handler(void) { Disable();while (1) {} }
void BusFault_Handler(void) { Disable();while (1) {} }
void UsageFault_Handler(void) { Disable();while (1) {} }
void Error_Handler(void) { Disable();__disable_irq();while (1) {} }
int _write(int fd,char *data,int size)
{
    (void)fd;
    if (HAL_UART_Transmit(&huart1,(uint8_t *)data,(uint16_t)size,100U)==HAL_OK) return size;
    Stop(M1_INPUT_FAULT);return -1;
}
static void Clock(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitTypeDef o={0};o.OscillatorType=RCC_OSCILLATORTYPE_HSI;o.HSIState=RCC_HSI_ON;
    o.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;o.PLL.PLLState=RCC_PLL_ON;
    o.PLL.PLLSource=RCC_PLLSOURCE_HSI;o.PLL.PLLM=8;o.PLL.PLLN=168;o.PLL.PLLP=RCC_PLLP_DIV2;o.PLL.PLLQ=4;
    if (HAL_RCC_OscConfig(&o)!=HAL_OK) Error_Handler();
    RCC_ClkInitTypeDef c={0};c.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;c.AHBCLKDivider=RCC_SYSCLK_DIV1;c.APB1CLKDivider=RCC_HCLK_DIV4;c.APB2CLKDivider=RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&c,FLASH_LATENCY_5)!=HAL_OK) Error_Handler();
}
static void InitStops(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();
    if(!M1Deadline_Prepare(M1_HOME_DEADLINE_MS))Error_Handler();
    HAL_NVIC_SetPriority(TIM5_IRQn,0,0);HAL_NVIC_EnableIRQ(TIM5_IRQn);
    DBGMCU->APB1FZ&=~(DBGMCU_APB1_FZ_DBG_TIM5_STOP|DBGMCU_APB1_FZ_DBG_IWDG_STOP);
    RCC->CSR|=RCC_CSR_LSION;uint32_t start=HAL_GetTick();
    while (!(RCC->CSR&RCC_CSR_LSIRDY)) if ((uint32_t)(HAL_GetTick()-start)>100U) Error_Handler();
    IWDG->KR=0xCCCC;IWDG->KR=0x5555;IWDG->PR=3;IWDG->RLR=499;
    start=HAL_GetTick();while (IWDG->SR) if ((uint32_t)(HAL_GetTick()-start)>100U) Error_Handler();
    IWDG->KR=0xAAAA; /* 典型约500ms，实际期限依LSI公差，尚未实测。 */
}
static void Sample(void)
{
    /* started在I2C读取前取得。若读取跨过HOME结束，不能把旧角度定为原点。 */
    uint32_t started=HAL_GetTick();M1Test before=Snapshot();
    M1Reference_ObserveState(&reference,&before,started);
    float angle;bool ok=Dev_Joint_ReadMotorAngle((DevJointId_t)axis->index,&angle)==ERR_OK && isfinite(angle);
    uint32_t key=__get_PRIMASK();__disable_irq();
    uint32_t finished=HAL_GetTick();
    feedback_valid=ok;feedback_seen=started;if(ok)motor_deg=angle;
    M1Test after=test;M1LimitSnapshot limits=ReadLimits();__set_PRIMASK(key);
    M1ReferenceSample sample={.started_ms=started,.finished_ms=finished,
        .motor_deg=ok?angle:0.0f,.valid=ok,.input_config_ok=limits.config_ok,
        .limit_mask=limits.active_mask};
    M1Reference_OnSample(&reference,&after,&sample);
}
static M1Test Snapshot(void)
{
    uint32_t key=__get_PRIMASK();__disable_irq();M1Test copy=test;__set_PRIMASK(key);return copy;
}
static void Start(bool home,bool high)
{
    if (!M1_HOME_MOTION_ENABLED) { puts("error: compiled output lock; action rejected\r");return; }
    if (M1Test_Active(&test) || test.used) { puts("error: one-shot session used; no retry\r");return; }
    /* 真实配置必须再次确认；UART不放入临界区，配置期间EN保持禁用。 */
    Disable();TmcConfigReport_t report;
    /* 内部电流参考，继续使用R110外部采样电阻；无需改变电位器。
     * IRUN8/IHOLD4为编码，按0.11欧/325mV计算约0.497/0.276A RMS，
     * 只是标称设定，未测绕组实际电流或负载所需力矩。 */
    driver_ready=Drv_TMC2209_ConfigNodeVerifiedWithReference(&huart6,axis->uart_node,16,8,4,
        TMC_CURRENT_INTERNAL_REF,&report);
    Sample();main_seen=HAL_GetTick();
    /* 配置有界阻塞期间，允许当前命令尾部的CR/LF到达，但不能把新的请求
     * 留到使能后才处理。溢出或其他待收字节一律拒绝本次动作，停止锁存。 */
    uint32_t key=__get_PRIMASK();__disable_irq();
    bool extra=!M1Rx_DrainTail(&rx);
    uint32_t sr=USART1->SR;extra|=(sr&(USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE))!=0U;
    while (USART1->SR&USART_SR_RXNE) {
        uint8_t ch=(uint8_t)USART1->DR;
        if (ch!='\r' && ch!='\n') extra=true;
    }
    if (extra) { Stop(M1_INPUT_FAULT);__set_PRIMASK(key);puts("error: data arrived during preflight; stopped\r");return; }
    uint32_t now=HAL_GetTick();M1LimitSnapshot limits=ReadLimits();M1Inputs in=Inputs(now,limits.active_mask);
    /* 不能只在诊断回复中报告配置错误，动作入口也必须拒绝错误的输入模式。 */
    bool ok=limits.config_ok && M1Test_Start(&test,now,home,high,M1_HOME_MOTION_ENABLED,driver_ready,&in);
    if (ok) {
        if(M1Deadline_Prepare(M1Test_Deadline(&test))){Apply();TIM5->CR1=TIM_CR1_CEN;}
        else {Disable();M1Test_Stop(&test,M1_INPUT_FAULT);ok=false;}
    }
    __set_PRIMASK(key);
    puts(ok?(SINGLE_AXIS_ID==1?"accepted: one-shot M1 test; lease required\r":
        "accepted: one-shot axis test; lease required\r"):"error: preflight rejected\r");
}
static void Handle(const char *line)
{
    M1Test s=Snapshot();
    if (!strcmp(line,"M112")) { Stop(M1_HOST_STOP);puts("M112OK\r");return; }
    if (!strcmp(line,"M115")) {
        /* 同时报告寻限位和退回界限，主机在HOME前逐项核对，不能只凭固件名启动。 */
        if(SINGLE_AXIS_ID==1)printf("M115 FIRMWARE:M1HomingPrepare PROTOCOL:M1HOME8 ");
        else printf("M115 FIRMWARE:AxisHomingPrepare PROTOCOL:"
#if M3_RETRY_500
                    "AXISHOME2"
#else
                    "AXISHOME1"
#endif
                    " AXIS:%u ",axis->id);
        printf("UART:115200 MOTION_COMPILED:%u AUTO_MOTION:0 JOG_STEPS:%u JOG_DEADLINE_MS:%u SEEK_STEPS:%u BACKOFF_STEPS:%u HOME_DEADLINE_MS:%u STEP_PERIOD_MS:%u",
            M1_HOME_MOTION_ENABLED,M1_JOG_STEPS,M1_JOG_DEADLINE_MS,
            M1_SEEK_STEPS,M1_BACKOFF_STEPS,M1_HOME_DEADLINE_MS,M1_PERIOD_MS);
#if M3_RETRY_500
        printf(" BACKOFF_PERIOD_MS:%u\r\n",M1_BACKOFF_PERIOD_MS);
#else
        puts("\r");
#endif
        return;
    }
    if (!strcmp(line,"M408")) {
        printf("M408 STATE:%u REASON:%u USED:%u EN:%s STEP:%u JOG:%lu SEEK:%lu BACKOFF:%lu DRIVER:%u DIR_CONFIRMED:%u HOMED:0 ALLOWED:0 RXERR:%lu RXOVF:%lu\r\n",
            s.state,s.reason,s.used,(GPIOD->IDR&GPIO_PIN_2)?"DISABLED":"ENABLED",
            ((GPIOA->IDR&axis->step_a)|(GPIOB->IDR&axis->step_b))!=0,
            (unsigned long)s.jog_steps,(unsigned long)s.seek_steps,(unsigned long)s.backoff_steps,driver_ready,s.direction_confirmed,
            (unsigned long)rx.uart_errors,(unsigned long)rx.overflows);return;
    }
    if (!strcmp(line,"M119")) {
        uint8_t mask=ReadLimits().active_mask;
        printf("M119 M1:%s M2:%s M3:%s\r\n",mask&1U?"TRIGGERED":"OPEN",mask&2U?"TRIGGERED":"OPEN",mask&4U?"TRIGGERED":"OPEN");return;
    }
    if (!strcmp(line,"M120")) {
        M1LimitSnapshot l=ReadLimits();
        printf("M120 RAW_PC_IDR:%08lX MODER:%08lX PUPDR:%08lX PC10:%u PC11:%u PC12:%u ACTIVE_MASK:%02X CONFIG_OK:%u LED_ON_PINS:%02lX\r\n",
            (unsigned long)l.idr,(unsigned long)l.moder,(unsigned long)l.pupdr,
            !(l.active_mask&1U),!(l.active_mask&2U),!(l.active_mask&4U),l.active_mask,l.config_ok,
            (unsigned long)((M1Limits_LedBsrr(l.active_mask)>>16U)&0x0eU));return;
    }
    if (!strcmp(line,"M121")) {
        uint32_t key=__get_PRIMASK();__disable_irq();M1LimitTrace t=limit_trace;__set_PRIMASK(key);
        printf("M121 SAMPLES:%lu LOW:%lu,%lu,%lu CHANGES:%lu,%lu,%lu LOW_SEEN_MASK:%02X\r\n",
            (unsigned long)t.samples,(unsigned long)t.low_samples[0],(unsigned long)t.low_samples[1],(unsigned long)t.low_samples[2],
            (unsigned long)t.changes[0],(unsigned long)t.changes[1],(unsigned long)t.changes[2],t.low_seen_mask);return;
    }
    if (!strcmp(line,"M114")) {
        printf("M114 RAW_M%u:%.3f VALID:%u AGE_MS:%lu\r\n",axis->id,(double)motor_deg,feedback_valid,(unsigned long)(HAL_GetTick()-feedback_seen));return;
    }
    if (!strcmp(line,"M122")) {
        /* 局部零点和整机HOMED严格分开；本命令只读，已用会话仍可查询。 */
        uint32_t now=HAL_GetTick();M1Reference_ObserveState(&reference,&s,now);
        printf("M122 LOCAL_REF:%u VALID:%u REASON:%u ORIGIN_MOTOR:%.3f REL_MOTOR:%.3f REL_JOINT:%.3f GEAR_RATIO:%.3f AGE_MS:%lu WHOLE_HOMED:0 ALLOWED:0\r\n",
            reference.state,reference.state==M1_REF_VALID,reference.reason,
            (double)reference.origin_motor_deg,(double)reference.relative_motor_deg,
            (double)reference.relative_joint_deg,(double)reference.gear_ratio,
            (unsigned long)(reference.state==M1_REF_VALID?now-reference.last_sample_started_ms:0U));return;
    }
    if (!strcmp(line,"KEEP")) {
        uint32_t key=__get_PRIMASK();__disable_irq();bool ok=M1Test_KeepAlive(&test,HAL_GetTick());__set_PRIMASK(key);
        puts(ok?"KEEP OK\r":"error: no live session\r");return;
    }
    if (s.used || M1Test_Active(&s)) { Stop(M1_INPUT_FAULT);puts("error: active or used session; command rejected\r");return; }
    if(!strcmp(line,"GUARDTEST")){
        if(M1_HOME_MOTION_ENABLED){puts("error: guard test requires compiled output lock\r");return;}
        Disable();uint32_t key=__get_PRIMASK();__disable_irq();
        bool ok=M1Deadline_Prepare(M1_JOG_DEADLINE_MS);
        uint32_t pending=HAL_NVIC_GetPendingIRQ(TIM5_IRQn),uif=TIM5->SR&TIM_SR_UIF;
        ok=ok && !pending && !uif && !(TIM5->CR1&TIM_CR1_CEN);
        if(!ok)M1Test_Stop(&test,M1_INPUT_FAULT);
        __set_PRIMASK(key);
        printf("GUARDTEST VALID:%u NVIC:%lu UIF:%lu ARR:%lu CR1:%lu\r\n",ok,
            (unsigned long)pending,(unsigned long)uif,(unsigned long)TIM5->ARR,(unsigned long)TIM5->CR1);return;
    }
    if (!strcmp(line,"CONFIG")) {
        TmcConfigReport_t report;
        driver_ready=Drv_TMC2209_ConfigNodeVerifiedWithReference(&huart6,axis->uart_node,16,8,4,
            TMC_CURRENT_INTERNAL_REF,&report);
        printf("CONFIG VALID:%u RESULT:%u REG:%02X READ_ERROR:%u CURRENT_REF:INTERNAL IRUN_CODE:8 IHOLD_CODE:4 GCONF:%08lX CHOPCONF:%08lX IFCNT:%u,%u\r\n",
            driver_ready,report.result,report.failed_register,report.read_error,
            (unsigned long)report.gconf_observed,(unsigned long)report.chopconf_observed,
            report.ifcnt_before,report.ifcnt_after);return;
    }
    if (!strcmp(line,"DIR D0") || !strcmp(line,"DIR D1")) {
        uint32_t key=__get_PRIMASK();__disable_irq();bool ok=M1Test_ConfirmDirection(&test,line[5]=='1');__set_PRIMASK(key);
        puts(ok?"DIR RECORDED: operator verification required\r":"error: direction rejected\r");return;
    }
    if (!strcmp(line,"JOG D0") || !strcmp(line,"JOG D1")) { Start(false,line[5]=='1');return; }
    if (!strcmp(line,"HOME")) { Start(true,false);return; }
    puts("error: M1 test command rejected\r");
}
int main(void)
{
    axis=AxisBinding_Get(SINGLE_AXIS_ID);
    SafeGPIO();HAL_Init();Clock();PlatformTime_Init();M1Test_Init(&test);
    M1Reference_Init(&reference,GEAR_RATIO);
    MX_USART6_UART_Init();MX_USART1_UART_Init();
    HAL_NVIC_DisableIRQ(USART1_IRQn);HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    MX_I2C1_Init();MX_I2C2_Init();MX_I2C3_Init();Dev_Joint_Init();SafeGPIO();
    setvbuf(stdout,NULL,_IONBF,0);InitStops();
    /* 不启动RX DMA；IRQ仅缓存字节，主循环保持严格单请求/回复。
     * 清除初始化期间的残留输入，所有动作仍由默认编译锁禁止。 */
    (void)USART1->SR;(void)USART1->DR;
    HAL_NVIC_SetPriority(USART1_IRQn,1,0);HAL_NVIC_EnableIRQ(USART1_IRQn);
    USART1->CR3|=USART_CR3_EIE;USART1->CR1|=USART_CR1_RXNEIE|USART_CR1_PEIE;
    char line[40];unsigned used=0;bool bad=false;uint32_t last_sample=HAL_GetTick()-20U,last_feed=HAL_GetTick();
    while (1) {
        uint32_t now=HAL_GetTick();main_seen=now;
        M1Test motion=Snapshot();M1Reference_ObserveState(&reference,&motion,now);
        if (now!=last_feed) { IWDG->KR=0xAAAA;last_feed=now; } /* SysTick停摆时不继续喂狗。 */
        if ((uint32_t)(now-last_sample)>=20U) { last_sample=now;Sample(); }
        uint8_t ch=0;uint32_t key=__get_PRIMASK();__disable_irq();
        bool fault=M1Rx_TakeFault(&rx);bool have=M1Rx_Pop(&rx,&ch);__set_PRIMASK(key);
        if(fault){Stop(M1_INPUT_FAULT);used=0;bad=true;continue;}
        if(!have)continue;
        if (ch=='\r' || ch=='\n') {
            if (bad) { Stop(M1_INPUT_FAULT);puts("error: malformed line\r"); }
            else if (used) { line[used]='\0';Handle(line); }
            used=0;bad=false;
        } else if (ch<32 || ch>126 || used>=sizeof(line)-1) bad=true;
        else if (!bad) line[used++]=(char)ch;
    }
}
