/**
 * @file main.c
 * @brief 仅供空载外接M1电机的单次UART台架；严禁接机械臂运行。
 * @details PD2为三驱动共用低有效EN，不能声称只使能M1；另外两电机须隔离。
 * 上电不运动；M791 UART内部转速或M792 STEP对照，共用一次1000ms锁。
 * M791时STEP全低；M792只发PA6的500个脉冲，不把它当UART写成功证据。
 * 停止先独立拉高EN并压低STEP，再发VACTUAL=0。
 * TIM5独立定时截止、SysTick截止、IWDG主循环看门狗三条停机路径。
 * 无PS2、回零、限位中断、I2C、运动服务或夹爪PWM。
 */
#include "stm32f4xx.h"
#include "bench_logic.h"
#include <stdio.h>
#include <string.h>
#if TMC_BENCH_LEGACY
#include "legacy_bridge.h"
#define BENCH_PROTOCOL "BENCH3"
#else
#define BENCH_PROTOCOL "BENCH2"
#endif

static volatile uint32_t ms, stop_reason, stop_pending, run_started, run_stopped;
static volatile uint32_t tx_count, rx_count, rx_last_count;
static volatile bool step_mode, step_level;
static volatile uint32_t emitted;
static uint32_t reset_flags;
static BenchGuard guard;
static char line[40];
static unsigned line_n;
static bool line_bad;
#if TMC_BENCH_LEGACY
uint32_t Bench_Milliseconds(void) { return ms; }
bool Bench_IsActive(void) { return guard.active; }
#endif

/* 所有停机路径只用BSRR原子写，不依赖UART、HAL锁或主循环。 */
static void Disable(void) { GPIOD->BSRR=1U<<2; GPIOA->BSRR=1U<<22; __DSB(); }
static void Trip(uint32_t reason)
{
    Disable();
    if (guard.active) {
        Bench_Stop(&guard); run_stopped=ms; stop_reason=reason; stop_pending=1;
    }
    TIM5->CR1=0;
    GPIOA->BSRR=1U<<22; step_level=false;
}
static void SafeOutputs(void)
{
    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOBEN|
                    RCC_AHB1ENR_GPIOCEN|RCC_AHB1ENR_GPIODEN;
    (void)RCC->AHB1ENR;
    Disable(); /* 在把PD2设为输出之前预置高电平。 */
    GPIOD->MODER=(GPIOD->MODER&~(3U<<4))|(1U<<4);
    /* DIR PD3固定低，建立时间远大于第一次STEP之前的配置/提示时间。 */
    GPIOD->BSRR=1U<<19;
    GPIOD->MODER=(GPIOD->MODER&~(3U<<6))|(1U<<6);
    GPIOA->BSRR=((1U<<6)|(1U<<7))<<16;
    GPIOA->MODER=(GPIOA->MODER&~((3U<<12)|(3U<<14)))|(1U<<12)|(1U<<14);
    GPIOB->BSRR=1U<<16; GPIOB->MODER=(GPIOB->MODER&~3U)|1U;
    /* PC0原理图LED1，低有效，作为台架存活指示。 */
    GPIOC->BSRR=1U<<16; GPIOC->MODER=(GPIOC->MODER&~3U)|1U;
}
static bool WaitBits(volatile uint32_t *reg,uint32_t mask,uint32_t expected)
{
    for(uint32_t n=0;n<4000000U;n++) if((*reg&mask)==expected)return true;
    return false;
}
static bool Clock168MHz(void)
{
    /* 保持正常固件HSI/8*168/2，UART1和UART6均APB2=84MHz。 */
    RCC->APB1ENR|=RCC_APB1ENR_PWREN; (void)RCC->APB1ENR;
    PWR->CR|=PWR_CR_VOS;
    RCC->CR|=RCC_CR_HSION;
    if(!WaitBits(&RCC->CR,RCC_CR_HSIRDY,RCC_CR_HSIRDY))return false;
    RCC->PLLCFGR=8U|(168U<<6)|(4U<<24);
    FLASH->ACR=FLASH_ACR_ICEN|FLASH_ACR_DCEN|FLASH_ACR_PRFTEN|FLASH_ACR_LATENCY_5WS;
    RCC->CFGR=RCC_CFGR_PPRE1_DIV4|RCC_CFGR_PPRE2_DIV2;
    RCC->CR|=RCC_CR_PLLON;
    if(!WaitBits(&RCC->CR,RCC_CR_PLLRDY,RCC_CR_PLLRDY))return false;
    RCC->CFGR=(RCC->CFGR&~RCC_CFGR_SW)|RCC_CFGR_SW_PLL;
    if(!WaitBits(&RCC->CFGR,RCC_CFGR_SWS,RCC_CFGR_SWS_PLL))return false;
    SystemCoreClock=168000000U; return true;
}
static void InitUart(void)
{
    RCC->APB2ENR|=RCC_APB2ENR_USART1EN|RCC_APB2ENR_USART6EN;
    (void)RCC->APB2ENR;
    GPIOA->MODER=(GPIOA->MODER&~((3U<<18)|(3U<<20)))|(2U<<18)|(2U<<20);
    GPIOA->OSPEEDR&=~((3U<<18)|(3U<<20)); /* 保持上位机GPIO LOW。 */
    GPIOA->AFR[1]=(GPIOA->AFR[1]&~((15U<<4)|(15U<<8)))|(7U<<4)|(7U<<8);
    GPIOC->MODER=(GPIOC->MODER&~((3U<<12)|(3U<<14)))|(2U<<12)|(2U<<14);
    GPIOC->OSPEEDR|=(3U<<12)|(3U<<14);
    GPIOC->AFR[0]=(GPIOC->AFR[0]&0x00FFFFFFU)|0x88000000U;
    USART1->BRR=729U; USART6->BRR=729U;
    USART1->CR1=USART6->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE;
}
static void InitStops(void)
{
    RCC->APB1ENR|=RCC_APB1ENR_TIM5EN; (void)RCC->APB1ENR;
    TIM5->PSC=8399U; TIM5->ARR=BENCH_RUN_MS*10U-1U; /* 84MHz/8400=10kHz。 */
    TIM5->EGR=TIM_EGR_UG; TIM5->SR=0; TIM5->DIER=TIM_DIER_UIE;
    NVIC_SetPriority(TIM5_IRQn,0); NVIC_EnableIRQ(TIM5_IRQn);
    SysTick_Config(SystemCoreClock/1000U); NVIC_SetPriority(SysTick_IRQn,1);
    /* 不冻结调试暂停时的看门狗/截止定时器；只有主循环健康才喂狗。 */
    DBGMCU->APB1FZ &= ~(DBGMCU_APB1_FZ_DBG_IWDG_STOP|DBGMCU_APB1_FZ_DBG_TIM5_STOP);
    RCC->CSR|=RCC_CSR_LSION;
    if(!WaitBits(&RCC->CSR,RCC_CSR_LSIRDY,RCC_CSR_LSIRDY))for(;;) { Disable(); }
    /* 与ST HAL_IWDG_Init顺序一致：先启动内核，再写PR/RLR并等待同步。
       只使能LSI不足以启动IWDG内核；先等待SR会永久卡住。 */
    IWDG->KR=0xCCCC;
    IWDG->KR=0x5555; IWDG->PR=3; IWDG->RLR=499; /* LSI典型32kHz/32，约500ms。 */
    uint32_t sync_start=ms;
    while(IWDG->SR)if((uint32_t)(ms-sync_start)>100U)for(;;) { Disable(); }
    IWDG->KR=0xAAAA;
}
void SysTick_Handler(void)
{
    ++ms;
    if(Bench_Expired(&guard,ms))Trip(1);
    else if(step_mode) {
        /* TIM5可抢占SysTick。仅把取状态/STEP原子写置于极短临界区，
           防止TIM5已停机后被抢占的SysTick又把STEP写高。 */
        uint32_t key=__get_PRIMASK(); __disable_irq();
        bool high=Bench_StepLevel(&guard,ms);
#if TMC_BENCH_LEGACY
        /* 备份原Drv_Stepper_Step中的HAL高/NOP/低真正执行。
           GPIO读回记录会增加少量脉宽，不声称完全复现原运动调度。 */
        if(high && !step_level) { Legacy_Pulse(); ++emitted; }
#else
        GPIOA->BSRR=high?(1U<<6):(1U<<22);
        if(high && !step_level)++emitted;
#endif
        step_level=high; __set_PRIMASK(key);
    }
}
void TIM5_IRQHandler(void) { TIM5->SR=0; Trip(2); }
void HardFault_Handler(void) { Disable(); for(;;) { } }
void MemManage_Handler(void) { Disable(); for(;;) { } }
void BusFault_Handler(void) { Disable(); for(;;) { } }
void UsageFault_Handler(void) { Disable(); for(;;) { } }
static bool WaitUart(USART_TypeDef *u,uint32_t bit)
{
    uint32_t start=ms;
    while(!(u->SR&bit)) if((uint32_t)(ms-start)>=5U)return false;
    return true;
}
static void Reply(const char *s)
{
    while(*s) {
        if(!WaitUart(USART1,USART_SR_TXE)) { Trip(5); return; }
        USART1->DR=(uint8_t)*s++;
    }
}
static void Delay(uint32_t duration)
{
    uint32_t start=ms; while((uint32_t)(ms-start)<duration) { }
}
static bool Send(const uint8_t *f,unsigned n)
{
    USART6->CR1&=~USART_CR1_RE;
    (void)USART6->SR; (void)USART6->DR;
    bool ok=true;
    for(unsigned k=0;k<n;k++) {
        if(!WaitUart(USART6,USART_SR_TXE)) { ok=false; break; }
        USART6->DR=f[k];
    }
    if(!WaitUart(USART6,USART_SR_TC))ok=false;
    (void)USART6->SR; (void)USART6->DR;
    USART6->CR1|=USART_CR1_RE;
    if(ok)++tx_count;
    return ok; /* TX成功仅指主控发完，绝不表示从机接受。 */
}
static bool Write(uint8_t n,uint8_t reg,uint32_t value)
{
    uint8_t f[8]; Bench_WriteFrame(n,reg,value,f);
    bool ok=Send(f,8); Delay(5); return ok;
}
static void ReadNode(uint8_t n)
{
    uint8_t f[]={5,n,6,0},r[8]; f[3]=Bench_Crc(f,3);
    bool sent=Send(f,4); unsigned count=0; uint32_t start=ms;
    while(count<8 && (uint32_t)(ms-start)<30U) {
        uint32_t sr=USART6->SR;
        if(sr&(USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)) {
            (void)USART6->DR; break;
        }
        if(sr&USART_SR_RXNE)r[count++]=(uint8_t)USART6->DR;
    }
    rx_last_count=count; rx_count+=count;
    bool valid=count==8 && r[0]==5 && r[1]==255 && r[2]==6 && Bench_Crc(r,7)==r[7];
    char out[150];
    uint32_t v=valid?((uint32_t)r[3]<<24)|((uint32_t)r[4]<<16)|((uint32_t)r[5]<<8)|r[6]:0;
    snprintf(out,sizeof(out),"M123 NODE:%u SENT:%u RX:%u VALID:%u IOIN:%08lX\r\n",
             n,sent,count,valid,(unsigned long)v); Reply(out);
}
static void Begin(bool external_step)
{
#if TMC_BENCH_LEGACY
    const char *verb="M797";
    (void)external_step; external_step=true;
#else
    const char *verb=external_step?"M792":"M791";
#endif
    if(guard.used) { Reply(verb); Reply(" REJECT:USED\r\n"); return; }
    Disable();
    bool ok=true;
    /* 清除所有地址可能遗留的内部速度，EN仍高；不写OTP或地址选择。 */
    for(uint8_t n=0;n<4;n++) if(!Write(n,0x22,0))ok=false;
#if TMC_BENCH_LEGACY
    if(!Legacy_ConfigOriginal())ok=false;
#else
    /* M1固定地址0。请求内部基准/外部R110、256微步，vsense=1，IRUN7/IHOLD3。
       约0.245Arms仅在所有配置确实接受时成立；本台架不把TX_OK当读回。 */
    if(!Write(0,0x00,0x1C0U))ok=false;
    if(!Write(0,0x03,0x200U))ok=false;
    if(!Write(0,0x10,0x60703U))ok=false;
    /* STEP对照请求16微步，实际接受未知；不据此承诺实际转角。 */
    if(!Write(0,0x6C,external_step?0x14020053U:0x10020053U))ok=false;
    if(!Write(0,0x22,external_step?0U:BENCH_VELOCITY))ok=false;
#endif
    if(!ok) { guard.used=true; Trip(4); Write(0,0x22,0); Reply(verb); Reply(" FAILED:TX EN:DISABLED\r\n"); return; }
    /* 先完成配置和主机提示；截止时间只覆盖真正EN低的区间。 */
#if TMC_BENCH_LEGACY
    Reply("M797 START AXIS:1 SOURCE:ORIGINAL_ZIP STEP_HZ:500 MAX_MS:1000 MAX_PULSES:500 IRUN:28 IHOLD:15\r\n");
#else
    if(external_step)Reply("M792 START AXIS:1 STEP_HZ:500 MAX_MS:1000 MAX_PULSES:500 CURRENT:REQUEST_ONLY\r\n");
    else Reply("M791 START NODE:0 VACTUAL:6000 MAX_MS:1000 STEP:LOW CURRENT:REQUEST_ONLY\r\n");
#endif
    if(!WaitUart(USART1,USART_SR_TC)) { guard.used=true; Write(0,0x22,0); return; }
    uint32_t key=__get_PRIMASK(); __disable_irq();
    step_mode=external_step; step_level=false; emitted=0;
    if(Bench_Start(&guard,ms)) {
        run_started=ms; TIM5->CNT=0; TIM5->SR=0; TIM5->CR1=TIM_CR1_OPM|TIM_CR1_CEN;
#if TMC_BENCH_LEGACY
        Legacy_Enable(); /* 调用备份原使能函数；所有截止仍独立用BSRR禁用。 */
#else
        GPIOD->BSRR=1U<<18; /* 唯一EN使能点；STEP仅在M792模式开始后由SysTick输出。 */
#endif
    }
    __set_PRIMASK(key);
}
static void Status(void)
{
    char s[240];
    snprintf(s,sizeof(s),"M408 EN:%s ACTIVE:%u USED:%u START:%lu STOP:%lu REASON:%lu MS:%lu STEP:%lu TX:%lu RX:%lu RESET:%08lX MODE:%s EMITTED:%lu\r\n",
        (GPIOD->ODR&4U)?"DISABLED":"ENABLED",guard.active,guard.used,
        (unsigned long)run_started,(unsigned long)run_stopped,(unsigned long)stop_reason,
        (unsigned long)ms,(unsigned long)(((GPIOA->ODR>>6)&3U)|((GPIOB->ODR&1U)<<2)),
        (unsigned long)tx_count,(unsigned long)rx_count,(unsigned long)reset_flags,
        step_mode?"STEP":"UART",(unsigned long)emitted);
    Reply(s);
}
static void TimerSelfTest(void)
{
    if(guard.used) { Reply("M794 REJECT:USED\r\n"); return; }
    Disable();
    /* 在不使能电机的条件下，让TIM5在200ms先于SysTick的1000ms截止。
       验证实际定时器中断路径；随后用M793验证独立看门狗及安全重启。 */
    Reply("M794 TIMER_TEST EN:DISABLED MAX_MS:200\r\n");
    uint32_t key=__get_PRIMASK(); __disable_irq();
    if(Bench_Start(&guard,ms)) {
        /* UG本身也是更新事件，必须先关UIE，避免把装载PSC/ARR误作截止。 */
        TIM5->DIER=0; run_started=ms; TIM5->ARR=1999U; TIM5->EGR=TIM_EGR_UG;
        TIM5->CNT=0; TIM5->SR=0; __DSB(); NVIC_ClearPendingIRQ(TIM5_IRQn);
        TIM5->DIER=TIM_DIER_UIE; TIM5->CR1=TIM_CR1_OPM|TIM_CR1_CEN;
    }
    __set_PRIMASK(key);
}
static void Command(const char *s)
{
    if(!strcmp(s,"M115"))Reply("M115 FIRMWARE:TmcExternalBench PROTOCOL:" BENCH_PROTOCOL " UART:115200 AUTO_MOTION:0\r\n");
    else if(!strcmp(s,"M408"))Status();
    else if(!strcmp(s,"M112")) { Trip(3); Disable(); Reply("M112 EN:DISABLED\r\n"); }
#if TMC_BENCH_LEGACY
    else if(!strcmp(s,"M797"))Begin(true);
    else if(!strcmp(s,"M798")) {
        char out[240];
        snprintf(out,sizeof(out),"M798 INIT:%lu CONFIG_TX:%lu TX_FAIL:%lu CONFIG:%lu HIGH:%lu LOW:%lu EN_LOW:%lu PULSES:%lu GCONF:%08lX CHOPCONF:%08lX CURRENT:%08lX\r\n",
          (unsigned long)g_legacy_report.initialized,(unsigned long)g_legacy_report.tx_count,
          (unsigned long)g_legacy_report.tx_fail,(unsigned long)g_legacy_report.config_count,
          (unsigned long)g_legacy_report.high_seen,(unsigned long)g_legacy_report.low_seen,
          (unsigned long)g_legacy_report.en_low_seen,(unsigned long)g_legacy_report.pulses,
          (unsigned long)g_legacy_report.values[0],(unsigned long)g_legacy_report.values[1],(unsigned long)g_legacy_report.values[2]);
        Reply(out);
    }
#else
    else if(!strcmp(s,"M791"))Begin(false);
    else if(!strcmp(s,"M792"))Begin(true);
#endif
    else if(!strcmp(s,"M794"))TimerSelfTest();
    else if(!strcmp(s,"M793") && !guard.active) {
        Disable(); Reply("M793 WATCHDOG_TEST EN:DISABLED\r\n");
        WaitUart(USART1,USART_SR_TC); for(;;) { } /* 显式、仅禁用态测试IWDG复位。 */
    }
    else if(strlen(s)==7 && !strncmp(s,"M123 N",6) && s[6]>='0' && s[6]<='3' && !guard.active)ReadNode((uint8_t)(s[6]-'0'));
    else Reply("ERROR COMMAND_REJECTED\r\n");
}
static void PollConsole(void)
{
    uint32_t sr=USART1->SR;
    if(sr&(USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)) {
        (void)USART1->DR; line_bad=true; Trip(6); return;
    }
    if(!(sr&USART_SR_RXNE))return;
    char c=(char)USART1->DR;
    if(c=='\r')return;
    if(c=='\n') {
        line[line_n]=0;
        if(line_n && !line_bad)Command(line);
        line_n=0; line_bad=false;
    } else if(line_n<sizeof(line)-1 && !line_bad)line[line_n++]=c;
    else { line_bad=true; Trip(6); }
}
int main(void)
{
    SafeOutputs(); reset_flags=RCC->CSR; RCC->CSR|=RCC_CSR_RMVF;
    if(!Clock168MHz())for(;;) { Disable(); }
    InitUart(); InitStops();
#if TMC_BENCH_LEGACY
    if(!Legacy_Init())for(;;) { Disable(); }
#endif
    Reply("# " BENCH_PROTOCOL " IDLE EN:DISABLED STEP:LOW NO_AUTO_RUN\r\n");
    for(;;) {
        /* 超时先已由ISR停EN，这里只补发UART停止及报告；串口坏也能停机。 */
        if(stop_pending) {
            stop_pending=0; Write(0,0x22,0);
            Reply("# STOPPED EN:DISABLED\r\n");
        }
        PollConsole(); IWDG->KR=0xAAAA;
    }
}
